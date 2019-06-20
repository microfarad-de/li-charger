/* 
 * Lithium Battery Charger
 * 
 * This source file is part of the Lithium-Ion Battery Charger Arduino firmware
 * found under http://www.github.com/microfarad-de/li-charger
 * 
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 * 
 * Copyright (C) 2019 Karim Hraibi (khraibi at gmail.com)
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 * 
 * Version: 2.2.0
 * Date:    May 2019
 */
#define VERSION_MAJOR 2  // major version
#define VERSION_MINOR 2  // minor version
#define VERSION_MAINT 0  // maintenance version

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include "CLI.h"
#include "ADC.h"
#include "Helper.h"
#include "Trace.h"


/*
 * Pin assignment
 */
#define NUM_APINS           2  // Number of analog pins in use
#define VOLTAGE_APIN ADC_PIN0  // Analog pin for voltage measurement
#define CURRENT_APIN ADC_PIN1  // Analog pin for current measurement
#define MOSFET_PIN          9  // PWM pin controlling the gate of the power MOSFET
#define LED_PIN            13   // LED pin


/*
 * Configuration parameters
 */
#define SERIAL_BAUD      115200 // Serial communication baud rate
#define V1_REF          4200000 // 4.20 V - Calibrate V1 against this reference voltage per cell in µV
#define V2_REF          1000000 // 1.00 V - Calibrate V2 against this reference voltage in µV
#define V_SURGE         4250000 // 4.25 V - maximum allowed surge voltage threshold per cell in µV
#define V_MAX           4190000 // 4.19 V - Maximum allowed battery voltage per cell in µV
#define V_MIN           2500000 // 2.50 V - Minimum allowed battery voltage per cell in µV
#define V_START          500000 // 0.50 V - Minimum allowed battery voltage per cell in µV for starting a charge
                                //          Use very low value for overcoming BMS protection and deep-dicharged NiCd cells (makeshift NiCd support)
#define V_SAFE          2800000 // 2.80 V - Chare with reduced current I_safe below this voltage per cell in µV
#define V_WINDOW           2000 // 0.002 V - Do not regulate voltage when within +/- this window (per cell) in µV
#define V_TRICKLE_START 4100000 // 4.10 V - Trickle charge threshold voltage in µV
#define V_TRICKLE_MAX   4150000 // 4.15 V - Trickle charge maximum voltage in µV
#define I_WINDOW          15000 // 0.015 A - Do not regulate current when within +/- this window in µA
#define I_DIVIDER           128 // Divider constant used for current calculation
#define I_SAFE_DIVIDER       10 // Divide I_chrg by this value to calculate I_safe, which is the reduced safety charging current
#define TIMEOUT_CHARGE     2000 // Time duration in ms during which V shall be between V_MIN and V_MAX before starting to charge
#define TIMEOUT_ERROR       150 // Time duration in ms during which i or V shall be out of bounds in order to trigger an error condition
#define TIMEOUT_FULL      20000 // Time duration in ms during which I_full shall not be exceeded in order to assume that battery is full
#define TIMEOUT_TRICKLE    5000 // Time duration in ms during which V shall be smaller than V_TRICKLE_MAX before starting a trickle charge
#define TIMEOUT_ERR_RST    2000 // Time duration in ms during which V shall be 0 before going back from STATE_ERROR to STATE_INIT
#define TIMEOUT_FULL_RST   2000 // Time duration in ms during which V shall be 0 before going back from STATE_FULL to STATE_INIT
#define TIMEOUT_UPDATE_UP    50 // Time interval in ms for increasing the power output by one increment
#define TIMEOUT_UPDATE_DN     1 // Time interval in ms for decreasing the power output by one increment
#define PWM_OC_DETECT_THR   150 // PWM value threshold - detect open circuit if I = 0 while PWM exceeds this value
#define ADC_AVG_SAMPLES      16 // Number of ADC samples to be averaged
#define SOC_LUT_SIZE          9 // Size of the state-of-charge lookup table
#define TRACE_BUF_SIZE      240 // Trace buffer size
#define TRACE_RESOLUTION  60000 // Trace timestamp resolution in ms




/* 
 * Objects
 */
LedClass Led;
TraceClass Trace;

/*
 * State machine states
 */
enum State_t { STATE_INIT_E, STATE_INIT, STATE_CHARGE_E, STATE_CHARGE, STATE_FULL_E, STATE_FULL, 
                STATE_ERROR_E, STATE_ERROR, STATE_CALIBRATE_E, STATE_CALIBRATE };

/*
 * Global variables
 */
struct {
  State_t state = STATE_INIT_E; // Current state machine state
  uint32_t v1;           // V1 - Voltage at the battery '+' terminal (MOSFET drain) in µV
  uint32_t v2;           // V2 - Voltage at the battery '-' terminal (shunt) in µV
  uint32_t v;            // V  - Battery voltage = V1 - V2 in µV
  uint32_t vMax;         // Maximum allowed battery voltage during charging in µV
  uint32_t i;            // I - Charging current in µA
  uint32_t iMax;         // I_max - Maximum charging current in µA
  uint32_t tMax;         // T_max - Maximum allowed charge time duration in s
  uint32_t iCalibration; // Calibration value for calculating i
  uint32_t c = 0;        // Total charged capacity in mAs
  uint32_t cMax;         // C_max - Maximum allowed charge capacity in mAs 
  uint32_t t = 0;        // Charge duration in s
  uint32_t tUpdate;      // Regulation loop update interval in ms
  uint16_t v1Raw;        // Raw ADC value of V1
  uint16_t v2Raw;        // Raw ADC value of V2
  uint16_t iSafe;        // I_safe - Charging current in mA when the battery voltage is below V_SAFE
  uint8_t dutyCycle;     // PWM duty cycle
  bool crcOk = false;    // EEPROM CRC check was successful
} G;



/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  uint32_t v1Calibration;       // V1_cal - Calibration value for calculating V1
  uint32_t v2Calibration;       // V2_cal - Calibration value for calculating V2
  uint16_t iFull;               // I_full - End of charge current in mA
  uint16_t iChrg;               // I_chrg - Maximum charging current in mA
  uint16_t rShunt;              // R_shunt - Shunt resistor value in mΩ
  uint16_t cFull;               // C_full - Full charge capacity in mAh
  uint8_t  numCells;            // N_cells - Number of Lithium-Ion cells
  uint16_t socLut[SOC_LUT_SIZE];// State-of-charge lookup table - contains voltages in mV 
  uint32_t crc;                 // CRC checksum
} Nvm;


/*
 * Strings to be reused for saving memory
 */
const struct {
  char *N_cells = (char *)"N_cells = %u\n";
  char *C_full  = (char *)"C_full  = %u mAh\n";
  char *I_chrg  = (char *)"I_chrg  = %u mA\n";
  char *I_safe  = (char *)"I_safe  = %u mA\n";
  char *I_full  = (char *)"I_full  = %u mA\n";
  char *R_shunt = (char *)"R_shunt = %u mΩ\n";
  char *V1_cal  = (char *)"V1_cal  = %lu\n";
  char *V2_cal  = (char *)"V2_cal  = %lu\n";
  char *I_cal   = (char *)"I_cal   = %lu\n";
  char *CRC     = (char *)"CRC = %lx\n\n";
  char *reached = (char *)" reached";
} Str;


/*
 * Validate the settings
 * Called after reading or before writing EEPROM
 * Always fall-back to the safest possible values
 */
void nvmValidate (void) {
  if (Nvm.numCells < 0 || Nvm.numCells > 6) Nvm.numCells = 1;
  if (Nvm.iChrg < 100 || Nvm.iChrg > 2000) Nvm.iChrg = 100;
  if (Nvm.iFull < 20 || Nvm.iFull > Nvm.iChrg - 20) Nvm.iFull = Nvm.iChrg - 20;
  if (Nvm.v1Calibration < 4000 || Nvm.v1Calibration > 40000) Nvm.v1Calibration = 40000;
  if (Nvm.v2Calibration < 800  || Nvm.v2Calibration > 1200 ) Nvm.v2Calibration = 1200;
  if (Nvm.cFull < 10 || Nvm.cFull > 10000 ) Nvm.cFull = 10;
  if (Nvm.rShunt < 100 || Nvm.rShunt > 1000) Nvm.rShunt = 1000;

  // Calculate current calibration value
  G.iCalibration = ((uint32_t)I_DIVIDER * 1000) / (uint32_t)Nvm.rShunt;

  // Calculate the safe charging current
  G.iSafe = Nvm.iChrg / (uint16_t)I_SAFE_DIVIDER;
}


/*
 * Read and validate EEPROM data
 */
void nvmRead (void) {
  uint32_t crc;
  
  eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm)); 
  nvmValidate ();

  // Calculate and check CRC checksum
  crc = crcCalc ((uint8_t*)&Nvm, sizeof (Nvm) - sizeof (Nvm.crc) );
  Cli.xprintf (Str.CRC, crc);

  if (crc != Nvm.crc) G.crcOk = false;
  else G.crcOk = true;
}


/*
 * Write and validate EEPROM data
 */
void nvmWrite (void) {
  nvmValidate (); 
  Nvm.crc = crcCalc ((uint8_t*)&Nvm, sizeof (Nvm) - sizeof (Nvm.crc) );
  //Cli.xprintf (Str.CRC, Nvm.crc);
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * Claculate the values of 
 * the maximum charge duration T_max and
 * the maximum charge capacity C_max
 */
void calcTmaxCmax (bool trickleCharge) {
  uint8_t i, soc;
  
  // Detect charge state using the lookup table
  for (i = 0; i < SOC_LUT_SIZE; i++) {
    if (G.v < (uint32_t)Nvm.socLut[i] * 1000 * (uint32_t)Nvm.numCells) break;
  }

  // Calculate the state of charge
  soc = (i * 100) / (SOC_LUT_SIZE + 1);
  
  //Cli.xprintf ("V   = %lu mV\n", G.v / 1000);
  Cli.xprintf ("SoC = %u %%\n", soc);
  Cli.xputs ("");
  
  /* Calculate the maximum allowed charge duration
     Assume linear increase with capacity intil 80% of charge (constant current phase), 
     then add a constant duration of 45 min for the remaining top-off charge (constant voltage phase).
     T_max (s) = 3600 * (C_full / I_chrg) * (90 - SoC) / 100 + 45 * 60s + T_safe
  */
  if (!trickleCharge) {
    // 36 = 3600 / 100
    G.tMax = ( 36 * (90 - soc) * (uint32_t)Nvm.cFull ) / (uint32_t)Nvm.iChrg + 45 * 60;
  }
  else {
    // Allow for 20 charging minutes in every trickle charge cycle
    G.tMax = 20 * 60 + G.t;
  }

  /* Calculate the maximum allowed charge capacity
     C_max (mAs) = 3600 * C_full * 1.31 * (100 - SoC) / 100
     Equals remaining capacity + 31%: 47 = 3600 * 1.31 / 100
  */
  if (!trickleCharge) {
    G.cMax = 47 * (100 - soc) * (uint32_t)Nvm.cFull;
  }
  else {
    // Top-up 3% of C_full for every trickle charge cycle
    G.cMax = 36 * 3 * (uint32_t)Nvm.cFull + G.c;
  }
  Trace.log ('%', soc);
  Trace.log ('v', G.v / 1000);
  Trace.log ('T', G.tMax / 60);
  Trace.log ('C', G.cMax / 3600);  
}




/*
 * Arduino initalization routine
 */
void setup (void) {

  MCUSR = 0;      // clear MCU status register
  wdt_disable (); // and disable watchdog

  // Initialize the command-line interface
  Cli.init (SERIAL_BAUD);
  Cli.xputs ("");
  Cli.xputs ("+ + +  L I  C H A R G E R  + + +");
  Cli.xputs ("");
  Cli.xprintf ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.xputs ("");
  Cli.newCmd ("ncells", "set N_cells", numCellsSet);
  Cli.newCmd ("cfull", "set C_full in mAh", cFullSet);
  Cli.newCmd ("ichrg", "set I_chrg in mA", iChrgSet);
  Cli.newCmd ("ifull", "set I_full in mA", iFullSet);
  Cli.newCmd ("rshunt", "set R_shunt in mΩ", rShuntSet);
  Cli.newCmd ("lut", "set LUT (lut <idx> <mV>)", socLutSet);
  Cli.newCmd ("c", "show cal. params", showCalibration);
  Cli.newCmd (".", "show r/t params", showRtParams);
  Cli.newCmd ("cal", "cal. V1 & V2 (cal [v1|v2])", calibrate);
  Cli.newCmd ("t", "trace", traceDump);
  
  Cli.showHelp ();

  // Initialize the ADC
  Adc.initialize (ADC_PRESCALER_128, ADC_INTERNAL, NUM_APINS, ADC_AVG_SAMPLES);

  // Initialize digital pins
  pinMode (MOSFET_PIN, OUTPUT);

  // Initialize LED
  Led.initialize (LED_PIN);

  // Read the settngs from EEPROM and validate them
  nvmRead ();

  // Initialize the trace buffer
  Trace.initialize (sizeof(Nvm), TRACE_BUF_SIZE, TRACE_RESOLUTION);

  // Enable the watchdog
  wdt_enable (WDTO_1S);

}


/*
 * Arduino main loop
 */
void loop (void) {
  static uint32_t tickTs = 0;
  static uint32_t updateTs = 0;
  static uint32_t chargeTs = 0;
  static uint32_t errorTs = 0;
  static uint32_t fullTs = 0;
  static uint32_t trickleTs = 0;
  static uint32_t resetTs = 0;
  static uint16_t traceCount = 0;
  static bool traceSurgeFlag = false;
  static bool safeCharge = true;
  static bool trickleCharge = false;
  uint32_t ts = millis ();

  // Reset the watchdog timer
  wdt_reset ();

  // Enter the power save mode - reduces accuracy of c calculation
  //powerSave ();
  
  // Command-line interpreter
  Cli.getCmd ();

  // Update the LED state
  Led.loopHandler ();

  // Update the trace timestamp - must be called after 'ts = millis ()'
  Trace.loopHandler ();

  // Read the ADC channels
  adcRead ();

  // Force the error STATE if CRC error occurred
  if (!G.crcOk && G.state != STATE_ERROR) Cli.xprintf ("CRC "), Trace.log ('E', 99), G.state = STATE_ERROR_E;


  // Main state machine
  switch (G.state) {
  
    /********************************************************************/
    // Initialization State
    case STATE_INIT_E:
      Led.blink (-1, 500, 1500);
      trickleCharge = false;
      Trace.reset ();
      G.vMax = (uint32_t)V_MAX * Nvm.numCells;
      chargeTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("Waiting for battery\n");
      G.state = STATE_INIT;
    case STATE_INIT:
      // Start charging if V stays within bounds during TIMEOUT_CHARGE
      if ( G.v < (uint32_t)V_START * Nvm.numCells || G.v > G.vMax ) chargeTs = ts;
      if (ts - chargeTs > TIMEOUT_CHARGE) {
        G.c = 0;
        G.t = 0;
        G.state = STATE_CHARGE_E;
      }
      break;

    /********************************************************************/
    // Charging State
    case STATE_CHARGE_E:
      Led.turnOn ();
      errorTs = ts;
      fullTs = ts;
      tickTs = ts;
      G.iMax = (uint32_t)G.iSafe * 1000;
      safeCharge = true;
      Trace.start ();
      traceCount = 0;
      if (trickleCharge) {
        G.vMax = (uint32_t)V_TRICKLE_MAX * Nvm.numCells; // Reduce vMax to trickle charge level
        Cli.xprintf ("Trickle c");
      }
      else  {
        G.vMax = (uint32_t)V_MAX * Nvm.numCells; // Set vMax to full charge level
        Cli.xprintf ("C");
      }
      Cli.xputs ("harging\n"); // No typo, just a trick to save memory
      Trace.log ('*', G.vMax / 1000);
      calcTmaxCmax (trickleCharge);
      if (G.v <= (uint32_t)V_SAFE * Nvm.numCells) Trace.log ('S', G.iMax / 1000);
      G.state = STATE_CHARGE;
      
    case STATE_CHARGE:

      // Temporarily increase the PWM update rate to mitigate voltage or current surge conditions
      if ( ( G.v > G.vMax + 10 * (uint32_t)V_WINDOW * Nvm.numCells ) ||
           ( G.i > G.iMax + 10 * (uint32_t)I_WINDOW ) ) {
        G.tUpdate = (uint32_t)TIMEOUT_UPDATE_DN;
      }
      else {
        G.tUpdate = (uint32_t)TIMEOUT_UPDATE_UP;
      }

      // CC-CV Regulation:
      // Run the regulation routine at the preset interval
      if (ts - updateTs > G.tUpdate) {
        updateTs = ts;

        // Regulate voltage and current with the CC-CV algorithm
        if ( ( G.v > G.vMax + (uint32_t)V_WINDOW * Nvm.numCells ) ||
             ( G.i > G.iMax + (uint32_t)I_WINDOW ) ) {
          if (G.dutyCycle > 0) G.dutyCycle--;
        }
        else if ( ( G.v < G.vMax - (uint32_t)V_WINDOW * Nvm.numCells ) &&
                  ( G.i < G.iMax - (uint32_t)I_WINDOW ) ) {
          if (G.dutyCycle < 255) G.dutyCycle++;
        }

        // Update the PWM duty cycle
        analogWrite (MOSFET_PIN, G.dutyCycle);
      }

      // Set the charging current
      if (G.v > (uint32_t)V_SAFE * Nvm.numCells && safeCharge) {
        safeCharge = false;
        G.iMax = (uint32_t)Nvm.iChrg * 1000;
        Trace.log ('I', G.iMax / 1000);
      }

      // Error Detection:
      // Signal an error if V stays out of bounds or open circuit condition occurs during TIMEOUT_ERROR
      if ( (G.v > (uint32_t)V_MIN * Nvm.numCells || safeCharge) && 
           G.v < (uint32_t)V_SURGE * Nvm.numCells  && 
           !( G.i == 0 && G.dutyCycle > PWM_OC_DETECT_THR ) ) errorTs = ts;
      if (ts - errorTs > TIMEOUT_ERROR) {
        showRtParams (0, NULL);
        if (G.v > (uint32_t)V_SURGE * Nvm.numCells)       Cli.xprintf ("Overvolt "), Trace.log ('E', 1);
        if (G.v < (uint32_t)V_MIN * Nvm.numCells  )       Cli.xprintf ("Undervolt "), Trace.log ('E', 2);
        if (G.i == 0 && G.dutyCycle > PWM_OC_DETECT_THR)  Cli.xprintf ("Open circuit "), Trace.log ('E', 3);
        G.state = STATE_ERROR_E;
      }


      // End of Charge Detection:
      // Report battery full if I_full has not been exceeded during TIMEOUT_FULL (ignore during safety charging)
      if ( G.i > (uint32_t)Nvm.iFull * 1000 || safeCharge ) fullTs = ts;
      if (ts - fullTs > TIMEOUT_FULL) {
        showRtParams (0, NULL);
        Cli.xprintf("I_full"); 
        Cli.xputs(Str.reached);
        Trace.log ('F', 1);
        G.state = STATE_FULL_E; 
      }

      // Calculate charged capacity by integrating i over time
      if (ts - tickTs >= 1000 /* && !trickleCharge */) {
        tickTs += 1000;
        G.t++;
        G.c += (G.i / 1000);
        traceCount++;
        if (traceCount >= 120) {
          Trace.log ('v', G.v / 1000);
          Trace.log ('i', G.i / 1000);
          traceCount = 0;
        }
      }

      // End of Charge Detection:
      // Maximum charge capacity is reached 
      if (G.c > G.cMax) {
        showRtParams (0, NULL); 
        Cli.xprintf ("C_max"); Cli.xputs(Str.reached);
        Trace.log ('F', 2);
        G.state = STATE_FULL_E;
      }
      
      // End of Charge Detection:
      // Maximum charge duration reached
      if (G.t > G.tMax) {
        showRtParams (0, NULL);
        Cli.xprintf ("T_max"); Cli.xputs(Str.reached);
        Trace.log ('F', 3);
        G.state = STATE_FULL_E;  
      }
      break;

    /********************************************************************/
    // Battery Full State
    case STATE_FULL_E:
      Led.blink (-1, 100, 1900);
      trickleCharge = true;
      trickleTs = ts;
      resetTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("Battery full\n");
      Trace.log ('t', G.t / 60);
      Trace.log ('c', G.c / 3600);
      Trace.log ('v', G.v / 1000);
      Trace.log ('i', G.i / 1000);
      G.state = STATE_FULL;     
    case STATE_FULL:
      // Start a trickle charging cycle if V_TRICKLE_START has not been exceeded during TIMEOUT_TRICKLE
      if (G.v > (uint32_t)V_TRICKLE_START * Nvm.numCells) trickleTs = ts;
      if (ts - trickleTs > TIMEOUT_TRICKLE) {
        showRtParams (0, NULL);
        G.state = STATE_CHARGE_E;
      }

      // Go to STATE_INIT if V stayed 0 during TIMEOUT_RESET
      if (G.v > 0) resetTs = ts;
      if (ts - resetTs > TIMEOUT_FULL_RST) G.state = STATE_INIT_E;
      break;
        
    /********************************************************************/
    // Error State
    case STATE_ERROR_E:
      Led.blink (-1, 200, 200);
      resetTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("ERROR\n");
      Trace.log ('t', G.t / 60);
      Trace.log ('c', G.c / 3600);
      Trace.log ('v', G.v / 1000);
      Trace.log ('i', G.i / 1000);
      G.state = STATE_ERROR;   
    case STATE_ERROR:
      // Go to STATE_INIT if V stayed 0 during TIMEOUT_RESET
      if (G.v > 0) resetTs = ts;
      if (ts - resetTs > TIMEOUT_ERR_RST && G.crcOk) G.state = STATE_INIT_E;
      break;

      
    /********************************************************************/
    // Calibration State
    case STATE_CALIBRATE_E:
      Led.blink (-1, 100, 100);
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("Calibration mode\n");
      G.state = STATE_CALIBRATE;   
    case STATE_CALIBRATE:
      // Do nothing and wait for a CLI command
      break;
      
    /********************************************************************/
    default:
      break;
  }
}






/*
 * Read the ADC channels
 */
void adcRead (void) {
  bool result;

  // Read the ADC channels
  result = Adc.readAll ();

  
  if (result) {
    // Get the ADC results
    G.v1Raw = (uint16_t)Adc.result[VOLTAGE_APIN];
    G.v2Raw = (uint16_t)Adc.result[CURRENT_APIN];

    // Calculate voltage and current
    G.v1 = (uint32_t)G.v1Raw * Nvm.v1Calibration;
    G.v2 = (uint32_t)G.v2Raw * Nvm.v2Calibration;
    G.v  = G.v1 - G.v2;
    G.i  = ( (uint32_t)G.v2 * G.iCalibration ) / I_DIVIDER ;
  }
}



/*
 * Power-save routine, sends the CPU into sleep mode
 */
void powerSave (void) {

  // configure lowest sleep mode that keeps clk_IO for Timer 1 used for PWM generation
  set_sleep_mode (SLEEP_MODE_IDLE); 

  // enter sleep, wakeup will be triggered by the next millis() interrupt
  sleep_enable (); 
  sleep_cpu ();
  sleep_disable ();
}





/*
 * CLI command for showing the real-time parameters
 * These values are continuously updated during runtime.
 */
int showRtParams (int argc, char **argv) {
  uint32_t hour = G.t / 3600;
  uint32_t min = G.t / 60 - (hour * 60);
  uint32_t sec = G.t - (hour * 3600) - (min * 60);
  Cli.xprintf ("T      = %02u:%02u:%02u\n", (uint8_t)hour, (uint8_t)min, (uint8_t)sec); 
  Cli.xprintf ("C      = %lu mAh\n", G.c / 3600);
  Cli.xprintf ("V      = %lu mV\n", G.v / 1000);
  Cli.xprintf ("I      = %lu mA\n", G.i / 1000);
  Cli.xprintf ("T_max  = %lu min\n", G.tMax / 60);
  Cli.xprintf ("C_max  = %lu mAh\n", G.cMax / 3600);
  Cli.xprintf ("V_max  = %lu mV\n", G.vMax / 1000);
  Cli.xprintf ("I_max  = %lu mA\n", G.iMax / 1000);  
  Cli.xprintf ("PWM    = %u\n", G.dutyCycle);
  Cli.xprintf ("V1_raw = %u\n", G.v1Raw);
  Cli.xprintf ("V2_raw = %u\n", G.v2Raw);
  Cli.xprintf ("V1     = %lu mV\n", G.v1 / 1000);
  Cli.xprintf ("V2     = %lu mV\n", G.v2 / 1000);
  Cli.xputs ("");
  return 0;
}



/*
 * CLI command for showing calibration parameters
 * These values do not change during runtime. 
 * They are either directly read from EEPROM or 
 * derived from other EEPROM values.
 */
int showCalibration (int argc, char **argv) {
  uint8_t i;
  Cli.xprintf (Str.N_cells, Nvm.numCells);
  Cli.xprintf (Str.C_full,  Nvm.cFull);
  Cli.xprintf (Str.I_chrg,  Nvm.iChrg);
  Cli.xprintf (Str.I_safe,  G.iSafe);
  Cli.xprintf (Str.I_full,  Nvm.iFull);
  Cli.xprintf (Str.R_shunt, Nvm.rShunt);
  Cli.xprintf (Str.V1_cal,  Nvm.v1Calibration);
  Cli.xprintf (Str.V2_cal,  Nvm.v2Calibration);
  Cli.xprintf (Str.I_cal,   G.iCalibration);
  Cli.xprintf ("LUT = ");
  for (i = 0; i < SOC_LUT_SIZE; i++) Cli.xprintf("%u ", Nvm.socLut[i]);
  Cli.xputs ("");
  Cli.xputs ("");
  return 0;
}


/*
 * CLI command for setting the number of cells
 * argv[1]: number of cells
 */
int numCellsSet (int argc, char **argv) {
  Nvm.numCells = atoi (argv[1]);
  nvmWrite ();
  Cli.xprintf (Str.N_cells, Nvm.numCells);
  Cli.xputs ("");
  G.state = STATE_INIT_E;
  return 0;
}


/*
 * CLI command for setting the charge current
 * argv[1]: current in mA
 */
int iChrgSet (int argc, char **argv) {
  Nvm.iChrg = atoi (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.I_chrg, Nvm.iChrg);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the end of charge current
 * argv[1]: current in mA
 */
int iFullSet (int argc, char **argv) {
  Nvm.iFull = atoi (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.I_full, Nvm.iFull);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the battery capacity
 * argv[1]: battery capacity in mAh
 */
int cFullSet (int argc, char **argv) {
  Nvm.cFull = atoi (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.C_full, Nvm.cFull);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the shunt resistor value
 * argv[1]: shunt resistance in mΩ
 */
int rShuntSet (int argc, char **argv) {
  Nvm.rShunt = atoi (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.R_shunt, Nvm.rShunt);
  Cli.xprintf(Str.I_cal, G.iCalibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for filling the state-of-charge lookup table
 * argv[1]: idx [0..8]
 *          V = LUT [idx]
 *          idx:    0     1     2     3     4     5     6     7     8
 *          SoC: 0% | 10% | 20% | 30% | 40% | 50% | 60% | 70% | 80% | 90%
 * argv[2]: voltage in mV per cell
 */
int socLutSet (int argc, char **argv) {
  uint8_t idx = (uint8_t)atoi (argv[1]);
  if (idx > SOC_LUT_SIZE - 1) return 1;
  Nvm.socLut[idx] = atoi (argv[2]);
  nvmWrite ();
  Cli.xprintf ("LUT[%u] = %u\n", idx, Nvm.socLut[idx]);
  Cli.xputs ("");
  return 0;
}


/*
 * Calibrate V1
 */
void calibrateV1 (void) {
  Nvm.v1Calibration = ((uint32_t)V1_REF * Nvm.numCells) / (uint32_t)G.v1Raw;
  nvmWrite ();
  Cli.xprintf (Str.V1_cal, Nvm.v1Calibration);
  Cli.xputs ("");
}


/*
 * Calibrate V2
 */
void calibrateV2 (void) {
  Nvm.v2Calibration = (uint32_t)V2_REF / (uint32_t)G.v2Raw;
  nvmWrite ();
  Cli.xprintf (Str.V2_cal, Nvm.v2Calibration);
  Cli.xputs ("");
}



/*
 * CLI command for calibrating V1 and V2
 * argv[1]:
 *   v1 : calibrate V1
 *   v2 : calibrate V2
 */
int calibrate (int argc, char **argv) {
  if (G.state == STATE_CALIBRATE) {
    if      (strcmp(argv[1], "v1") == 0) calibrateV1 ();
    else if (strcmp(argv[1], "v2") == 0) calibrateV2 ();
    else    G.state = STATE_INIT_E;
  }
  else {
    G.state = STATE_CALIBRATE_E;        
    Cli.xprintf ("V1_ref = %lu mV\n", (V1_REF * Nvm.numCells) / 1000);
    Cli.xprintf ("V2_ref = %lu mV\n\n", V2_REF / 1000);
  }
  return 0;
}


/*
 * CLI command for dumping the trace buffer
 */
int traceDump (int argc, char **argv) {
  Trace.dump ();
  return 0;
}
