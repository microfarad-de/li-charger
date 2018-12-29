/*
 * Lithium Battery Charger
 * 
 * Author:  Karim Hraibi
 * Version: 0.1.0
 * Date:    13.12.2018
 */
#define VERSION_MAJOR 0  // major version
#define VERSION_MINOR 1  // minor version
#define VERSION_MAINT 0  // maintenance version

#include <Arduino.h>
#include "cli.h"
#include "adc.h"
#include "helper.h"



#define NUM_APINS 2            // Number of analog pins in use
#define VOLTAGE_APIN ADC_PIN0  // Analog pin for voltage measurement
#define CURRENT_APIN ADC_PIN1  // Analog pin for current measurement

#define MOSFET_PIN 9           // PWM pin controlling the gate of the power MOSFET
#define LED_PIN 13             // LED pin

#define VOLT_REFERENCE 1000000 // 1.00 Volt - Reference voltage used for calibration in uV
#define VOLT_MAX       4180000 // 4.15 Volt - Maximum allowable battery voltage in uV
#define VOLT_MIN       2600000 // 2.60 Volt - Minimum allowable battery voltage in uV
#define VOLT_WINDOW      10000 // 0.01 Volt - Do not regulate when within +/- this window in uV
#define UPDATE_INTERVAL     50 // Interval for updating the power output in ms 
#define IIR_FILTER_TAPS      4 // IIR fiter taps for smoothing the ADC output

// Code snippets
#define EEPROM_READ() eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
#define EEPROM_WRITE() eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));


LedClass Led;    // LED object

/*
 * Global variables
 */
struct {
  uint8_t analogPin[NUM_APINS] = { VOLTAGE_APIN, CURRENT_APIN };  // Analog pins as an array
  int16_t v1Raw;  // Raw ADC value of v1
  int16_t v2Raw;  // Raw ADC value of v2
  int32_t v1;     // Voltage at the battery '+' terminal (MOSFET drain) in uV
  int32_t v2;     // Voltage at the battery '-' terminal (shunt) in uV
  int32_t i;      // Current
  int16_t dutyCycle;
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
  Cli.newCmd ("p", "set PWM: 0..255", cmdPwm);
  Cli.newCmd ("v1cal", "calibrate V1", v1Calibrate);
  Cli.newCmd ("v2cal", "calibrate V2", v2Calibrate);
  Cli.newCmd ("ical", "calibrate I", iCalibrate);
  Cli.newCmd ("c", "show calibration", showCalibration);
  Cli.newCmd (".", "show results", cmdResult);
  
  Cli.showHelp ();

  // Initialize the ADC
  ADConv.initialize (ADC_PRESCALER_128, ADC_INTERNAL, NUM_APINS);

  pinMode (MOSFET_PIN, OUTPUT);

  Led.initialize (LED_PIN);

  EEPROM_READ()

}


/*
 * Arduino main loop
 */
void loop (void) {
  static enum { STATE_INIT_E, STATE_INIT, STATE_CHARGE_E, STATE_CHARGE, STATE_FULL_E, STATE_FULL, STATE_ERROR_E, STATE_ERROR } state = STATE_INIT_E;
  static uint32_t updateTs = 0;
  int32_t vBatt;
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
      Led.blink (-1, 200, 800);
      G.dutyCycle = 0;
      state = STATE_INIT; 
      
    case STATE_INIT:
      state = STATE_CHARGE_E;
      break;


    /**********************************/
    // Charging
    case STATE_CHARGE_E:
      Led.turnOn ();
      state = STATE_CHARGE;
      
    case STATE_CHARGE:
      if (ts - updateTs > UPDATE_INTERVAL) {
        updateTs = ts;
        vBatt = G.v1 - G.v2;

        // Abort if voltage out of bounds
        //if (vBatt > ((int32_t)VOLT_MAX + 10*(int32_t)VOLT_WINDOW)) state = STATE_ERROR_E;
        //if (vBatt < VOLT_MIN)                 state = STATE_ERROR_E;

        // Regulate voltage
        if (vBatt > (int32_t)VOLT_MAX + (int32_t)VOLT_WINDOW) {
          if (G.dutyCycle > 0) G.dutyCycle--;
        }
        else if (vBatt < (int32_t)VOLT_MAX - (int32_t)VOLT_WINDOW) {
          if (G.dutyCycle < 255) G.dutyCycle++;
        }

        analogWrite (MOSFET_PIN, (int8_t)G.dutyCycle);  
      }
      
      break;

      
    /**********************************/
    // Battery full
    case STATE_FULL_E:
      Led.blink (-1, 200, 800);
      state = STATE_FULL;
      
    case STATE_FULL:

      break;
      
      
    /**********************************/
    // Error
    case STATE_ERROR_E:
      Led.blink (-1, 200, 200);
      analogWrite (MOSFET_PIN, 0);
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

  // Smoothen results
  if (result == 0) {
    G.v1Raw = v1Filter.process (ADConv.result[0], IIR_FILTER_TAPS);
    G.v2Raw = v2Filter.process (ADConv.result[1], IIR_FILTER_TAPS);

    G.v1 = (int32_t)G.v1Raw * Nvm.v1Calibration;
    G.v2 = (int32_t)G.v2Raw * Nvm.v2Calibration;
  }
}





/*
 * CLI command for printing the ADC results
 */
int cmdResult (int argc, char **argv) {
  int i;
  Cli.xprintf ("Duty Cyc = %d\n", G.dutyCycle);
  Cli.xprintf ("V1 (raw) = %d\n", G.v1Raw);
  Cli.xprintf ("V2 (raw) = %d\n", G.v2Raw);
  Cli.xprintf ("V1       = %ld\n", G.v1);
  Cli.xprintf ("V2       = %ld\n", G.v2);
  Cli.xprintf ("I        = %ld\n", G.i);
  Cli.xputs("");
  return 0;
}


/*
 * Show calibration values
 */
int showCalibration (int argc, char **argv) {
  Cli.xprintf("V1 cal. = %ld\n", Nvm.v1Calibration);
  Cli.xprintf("V2 cal. = %ld\n", Nvm.v2Calibration);
  Cli.xprintf("I  cal. = %ld\n", Nvm.iCalibration);
  Cli.xputs("");
  return 0;
}


/*
 * Calibrate V1
 */
int v1Calibrate (int argc, char **argv) {
  Nvm.v1Calibration = (int32_t)(5*VOLT_REFERENCE/G.v1Raw);
  EEPROM_WRITE()
  Cli.xprintf("V1 cal. = %ld\n", Nvm.v1Calibration);
  Cli.xputs("");
  return 0;
}


/*
 * Calibrate V2
 */
int v2Calibrate (int argc, char **argv) {
  Nvm.v2Calibration = (int32_t)(VOLT_REFERENCE/G.v2Raw);
  EEPROM_WRITE()
  Cli.xprintf("V2 cal. = %ld\n", Nvm.v2Calibration);
  Cli.xputs("");
  return 0;
}


/*
 * Calibrate I
 */
int iCalibrate (int argc, char **argv) {
  EEPROM_WRITE()
  Cli.xputs("");
  return 0;
}



/*
 * Set the duty cycle of the PWM output
 */
int cmdPwm (int argc, char **argv) {
  uint8_t val = atoi (argv[1]);
  Cli.xprintf ("%d\n", val);
  analogWrite (MOSFET_PIN, val);
  Cli.xputs("");
  return 0;
}
