/*
 * Lithium Battery Charger
 * 
 * Author:  Karim Hraibi
 * Version: 0.1.0
 * Date:    13.12.2018
 */
#define VERSION_MAJOR 1  // major version
#define VERSION_MINOR 0  // minor version
#define VERSION_MAINT 0  // maintenance version

#include <Arduino.h>
#include "cli.h"
#include "adc.h"
#include "helper.h"



#define NUM_APINS    2         // Number of analog pins in use
#define VOLTAGE_APIN ADC_PIN0  // Analog pin for voltage measurement
#define CURRENT_APIN ADC_PIN1  // Analog pin for current measurement

#define MOSFET_PIN      9      // PWM pin controlling the gate of the power MOSFET
#define LED_PIN        13      // LED pin
#define ADC_ENABLE_PIN  8      // Controls the BJT transistor bridge for connecting the ADC voltage divider to battery +
                               // ADC voltage divider must be disconnected to avoid battery drain

#define VOLT_CALIBRATION_V1 8400000 // 8.40 V - Calibrate V1 against this voltage in µV
#define VOLT_CALIBRATION_V2 1000000 // 1.00 V - Calibrate V2 against this voltage in µV
#define VOLT_MAX            8380000 // 8.38 V - Maximum allowed battery voltage in µV
#define VOLT_MIN            5200000 // 5.20 V - Minimum allowed battery voltage in µV
#define VOLT_WINDOW           20000 // 0.02 V - Do not regulate voltage when within +/- this window in µV
#define CURR_CALIBRATION_P      100 // Determines number of digits of the current calibration value (precision)
#define CURR_MAX            1000000 // 1.00 A - Maximum allowed current in µA
#define CURR_WINDOW           10000 // 0.01 A - Do not regulate current when within +/- this window in µA
#define CURR_FULL             50000 // 0.05 A - The battery is considered full if this current in µA is not exceeded during TIMEOUT_FULL
#define TIMEOUT_FULL          60000 // Time duration in ms during which CURR_FULL shall not be exceeded in order to assume that battery is full
#define UPDATE_INTERVAL          50 // Interval for updating the power output in ms 
#define IIR_FILTER_TAPS           2 // IIR fiter taps for smoothing the ADC output


// Code snippets
#define EEPROM_READ() eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
#define EEPROM_WRITE() eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));


// LED object
LedClass Led;

/*
 * Global variables
 */
struct {
  int32_t v1;        // V1 - Voltage at the battery '+' terminal (MOSFET drain) in µV
  int32_t v2;        // V2 - Voltage at the battery '-' terminal (shunt) in µV
  int32_t vBatt;     // Vbatt - Battery voltage = V1 - V2
  int32_t i;         // I - Current
  int16_t v1Raw;     // Raw ADC value of V1
  int16_t v2Raw;     // Raw ADC value of V2
  int16_t dutyCycle; // PWM duty cycle
} G;


/*
 * Parameters stored in EEPROM
 */
struct {
  int32_t v1Calibration;  // Calibration value for calculating v1
  int32_t v2Calibration;  // Calibration value for calculating v2
  int32_t iCalibration;   // Calibration value for calculating i
} Nvm;


/*
 * Arduino initalization routine
 */
void setup (void) {

  // Initialize the command-line interface
  Cli.init();
  Cli.xputs ("");
  Cli.xputs ("+ + +  L I T H I U M  C H A R G E R  + + +");
  Cli.xputs ("");
  Cli.xprintf ("Version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.xputs ("");
  Cli.newCmd ("v1cal", "calibrate V1", v1Calibrate);
  Cli.newCmd ("v2cal", "calibrate V2", v2Calibrate);
  Cli.newCmd ("ical", "calibrate I (shunt R in mΩ as arg)", iCalibrate);
  Cli.newCmd ("c", "show calibration", showCalibration);
  Cli.newCmd (".", "show voltage & current", cmdResult);
  Cli.showHelp ();

  // Initialize the ADC
  ADConv.initialize (ADC_PRESCALER_128, ADC_INTERNAL, NUM_APINS);

  // Initialize digital pins
  pinMode (MOSFET_PIN, OUTPUT);
  pinMode (ADC_ENABLE_PIN, OUTPUT);

  // Initialize LED
  Led.initialize (LED_PIN);

  // Read calibration data from EEPROM
  EEPROM_READ()

}


/*
 * Arduino main loop
 */
void loop (void) {
  static enum { STATE_INIT_E, STATE_INIT, STATE_CHARGE_E, STATE_CHARGE, 
      STATE_FULL_E, STATE_FULL, STATE_ERROR_E, STATE_ERROR } state = STATE_INIT_E;
  static uint32_t updateTs = 0;
  static uint32_t fullTs = 0;
  uint32_t ts = millis ();
  
  // Command-line interpreter
  Cli.getCmd ();

  // Update the LED state
  Led.loopHandler ();

  // Read the ADC channels
  adcRead ();


  // Main state machine
  switch (state) {

    
    /**********************************/
    // Initializing
    case STATE_INIT_E:
      Led.blink (-1, 400, 1600);
      digitalWrite (ADC_ENABLE_PIN, HIGH);
      G.dutyCycle = 0;
      Cli.xputs ("Waiting for battery\n");
      state = STATE_INIT; 
      
    case STATE_INIT:
      // Check if a battery is connected
      if ( G.vBatt > (int32_t)VOLT_MIN && G.vBatt < (int32_t)VOLT_MAX ) {
        state = STATE_CHARGE_E;
      }
      
      break;


    /**********************************/
    // Charging
    case STATE_CHARGE_E:
      Led.turnOn ();
      fullTs = ts;
      Cli.xputs ("Charging\n");
      state = STATE_CHARGE;
      
    case STATE_CHARGE:
      if (ts - updateTs > UPDATE_INTERVAL) {
        updateTs = ts;

        // Abort if voltage out of bounds
        if (G.vBatt > ((int32_t)VOLT_MAX + 4*(int32_t)VOLT_WINDOW)) state = STATE_ERROR_E;
        if (G.vBatt < (int32_t)VOLT_MIN)                            state = STATE_ERROR_E;

        // Regulate voltage and current
        if ( ( G.vBatt > (int32_t)VOLT_MAX + (int32_t)VOLT_WINDOW ) ||
             ( G.i     > (int32_t)CURR_MAX + (int32_t)CURR_WINDOW ) ) {
          if (G.dutyCycle > 0) G.dutyCycle--;
        }
        else if ( ( G.vBatt < (int32_t)VOLT_MAX - (int32_t)VOLT_WINDOW ) &&
                  ( G.i     < (int32_t)CURR_MAX - (int32_t)CURR_WINDOW ) ) {
          if (G.dutyCycle < 255) G.dutyCycle++;
        }

        analogWrite (MOSFET_PIN, (int8_t)G.dutyCycle);

        // Reset battery full timer
        if (G.i > (int32_t)CURR_FULL) fullTs = ts;

        // Battery full
        if (ts - fullTs > TIMEOUT_FULL) state = STATE_FULL_E;
      }
      
      break;

      
    /**********************************/
    // Battery full
    case STATE_FULL_E:
      Led.blink (-1, 100, 1900);
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      digitalWrite (ADC_ENABLE_PIN, LOW);
      Cli.xputs ("Battery full\n");
      state = STATE_FULL;
      
    case STATE_FULL:

      break;
      
      
    /**********************************/
    // Error
    case STATE_ERROR_E:
      Led.blink (-1, 200, 200);
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      digitalWrite (ADC_ENABLE_PIN, LOW);
      Cli.xputs ("ERROR\n");
      state = STATE_ERROR;
      
    case STATE_ERROR:

      break;

 
    /**********************************/
    default:
      break;
  }
}






/*
 * Read the ADC channels
 */
void adcRead (void) {
  int8_t result;
  static IirFilterClass v1Filter, v2Filter;

  // Read the ADC channels
  result = ADConv.readAll ();

  
  if (result == 0) {
    // Smoothen results
    G.v1Raw = v1Filter.process (ADConv.result[0], IIR_FILTER_TAPS);
    G.v2Raw = v2Filter.process (ADConv.result[1], IIR_FILTER_TAPS);

    // Calculate voltage and current
    G.v1 = (int32_t)G.v1Raw * Nvm.v1Calibration;
    G.v2 = (int32_t)G.v2Raw * Nvm.v2Calibration;
    G.vBatt = G.v1 - G.v2;
    G.i  = ( (int32_t)G.v2 * Nvm.iCalibration ) / CURR_CALIBRATION_P ;
  }
}





/*
 * CLI command for printing the ADC results
 */
int cmdResult (int argc, char **argv) {
  int i;
  Cli.xprintf ("Duty C.  = %d\n", G.dutyCycle);
  Cli.xprintf ("V1 (raw) = %d\n", G.v1Raw);
  Cli.xprintf ("V2 (raw) = %d\n", G.v2Raw);
  Cli.xprintf ("V1       = %ld\n", G.v1);
  Cli.xprintf ("V2       = %ld\n", G.v2);
  Cli.xprintf ("Vbatt    = %ld\n", G.vBatt);
  Cli.xprintf ("I        = %ld\n", G.i);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for showing calibration values
 */
int showCalibration (int argc, char **argv) {
  Cli.xprintf("V1 cal. = %ld\n", Nvm.v1Calibration);
  Cli.xprintf("V2 cal. = %ld\n", Nvm.v2Calibration);
  Cli.xprintf("I  cal. = %ld\n", Nvm.iCalibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for calibrating V1
 */
int v1Calibrate (int argc, char **argv) {
  Nvm.v1Calibration = (int32_t)VOLT_CALIBRATION_V1 / (int32_t)G.v1Raw;
  EEPROM_WRITE()
  Cli.xprintf("V1 cal. = %ld\n", Nvm.v1Calibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for calibrating V2
 */
int v2Calibrate (int argc, char **argv) {
  Nvm.v2Calibration = (int32_t)VOLT_CALIBRATION_V2 / (int32_t)G.v2Raw;
  EEPROM_WRITE()
  Cli.xprintf("V2 cal. = %ld\n", Nvm.v2Calibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for calibrating I
 * argv[1]: shunt resistance in milliohm
 */
int iCalibrate (int argc, char **argv) {
  int32_t r = atoi (argv[1]);
  Nvm.iCalibration = ((int32_t)CURR_CALIBRATION_P * 1000) / r;
  EEPROM_WRITE()
  Cli.xprintf("I cal. = %ld\n", Nvm.iCalibration);
  Cli.xputs("");
  return 0;
}
