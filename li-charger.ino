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
 * Version: 1.2.0
 * Date:    January 2019
 */
#define VERSION_MAJOR 1  // major version
#define VERSION_MINOR 2  // minor version
#define VERSION_MAINT 0  // maintenance version

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include "cli.h"
#include "adc.h"
#include "helper.h"


/*
 * Pin assignment
 */
#define NUM_APINS    2         // Number of analog pins in use
#define VOLTAGE_APIN ADC_PIN0  // Analog pin for voltage measurement
#define CURRENT_APIN ADC_PIN1  // Analog pin for current measurement
#define MOSFET_PIN      9      // PWM pin controlling the gate of the power MOSFET
#define LED_PIN        13      // LED pin


/*
 * Configuration parameters
 */
#define V1_REF          4200000 // 4.20 V - Calibrate V1 against this reference voltage per cell in µV
#define V2_REF          1000000 // 1.00 V - Calibrate V2 against this reference voltage in µV
#define V_SURGE         4250000 // 4.25 V - maximum allowed surge voltage threshold per cell in µV
#define V_MAX           4190000 // 4.19 V - Maximum allowed battery voltage per cell in µV
#define V_MIN            500000 // 0.50 V - Minimum allowed battery voltage per cell in µV for starting a charge
#define V_SAFE          2600000 // 2.60 V - Battery voltage threshold per cell in µV for charging at full current
#define V_WINDOW           2000 // 0.002 V - Do not regulate voltage when within +/- this window (per cell) in µV
#define V_DELTA           30000 // 0.03 V - Maximum momentary battery voltage drop per cell in µV
#define V_TRICKLE_START 4100000 // 4.10 V - Trickle charge threshold voltage in µV
#define V_TRICKLE_MAX   4150000 // 4.15 V - Trickle charge maximum voltage in µV
#define I_CALIBRATION_P     128 // Determines number of digits of the current calibration value (precision)
#define I_WINDOW          15000 // 0.015 A - Do not regulate current when within +/- this window in µA
#define I_DELTA          100000 // 0.1 A - Current increase in µA to detect end of charge
#define I_SAFE_FACTOR        20 // Divide I_chrg by this value to calculate I_safe, which is the reduced safety charging current
#define TIMEOUT_CHARGE     2000 // Time duration in ms during which vBatt shall be between V_MIN and V_MAX before starting to charge
#define TIMEOUT_ERROR       150 // Time duration in ms during which i or vBatt shall be out of bounds in order to trigger an error condition
#define TIMEOUT_FULL      20000 // Time duration in ms during which iFull shall not be exceeded in order to assume that battery is full
#define TIMEOUT_TRICKLE   10000 // Time duration in ms during which vBatt shall be smaller than V_TRICKLE_MAX before starting a trickle charge
#define TIMEOUT_ERR_RST    5000 // Time duration in ms during which vBatt shall be 0 before going back from STATE_ERROR to STATE_INIT
#define TIMEOUT_FULL_RST   2000 // Time duration in ms during which vBatt shall be 0 before going back from STATE_FULL to STATE_INIT
#define UPDATE_INTERVAL      50 // Interval in ms for updating the power output
#define ADC_AVG_SAMPLES      16 // Number of ADC samples to be averaged





/* 
 * LED object
 */
LedClass Led;

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
  uint32_t vBatt;        // Vbatt - Battery voltage = V1 - V2 in µV
  uint32_t vMax;         // Maximum allowed battery voltage during charging in µV
  uint32_t vMin;         // Minimum allowed battery voltage during charging in µV
  uint32_t i;            // I - Charging current in µA
  uint32_t iMax;         // I_max - Maximum charging current in µA
  uint32_t iMin;         // I_min - historically minimum charge current
  uint32_t iSafe;        // I_safe - Charging current in µA when the battery voltage is below V_SAFE
  uint32_t v1Raw;        // Raw ADC value of V1
  uint32_t v2Raw;        // Raw ADC value of V2
  uint16_t dutyCycle;    // PWM duty cycle
  uint32_t tMax;         // Maximum allowed charge time duration in s
  uint32_t iCalibration; // Calibration value for calculating i
  uint32_t c = 0;        // Total charged capacity in mAs
  uint32_t t = 0;        // Charge duration
  bool crcOk = false;    // EEPROM CRC check was successful
} G;



/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  uint32_t v1Calibration;  // V1_cal - Calibration value for calculating v1
  uint32_t v2Calibration;  // V2_cal - Calibration value for calculating v2
  uint32_t iFull;          // I_full - End of charge current in µA
  uint32_t numCells;       // N_cells - Number of Lithium-Ion cells
  uint32_t iChrg;          // I_chrg - Maximum charging current in µA
  uint32_t rShunt;         // R_shung - Shunt resistor value in mΩ
  uint32_t cMax;           // c_Max - Battery capacity in mAh
  uint32_t crc;            // CRC checksum
} Nvm;


/*
 * Strings to be reused for saving memory
 */
const struct {
  char *N_cells = (char *)"N_cells = %lu\n";
  char *C_max   = (char *)"C_max   = %lu mAh\n";
  char *I_chrg  = (char *)"I_chrg  = %lu mA\n";
  char *I_safe  = (char *)"I_safe  = %lu mA\n";
  char *I_full  = (char *)"I_full  = %lu mA\n";
  char *T_max   = (char *)"T_max   = %lu min\n";
  char *R_shunt = (char *)"R_shunt = %lu mΩ\n";
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
  if (Nvm.iChrg < 100000 || Nvm.iChrg > 5000000) Nvm.iChrg = 100000;
  if (Nvm.iFull < 20000 || Nvm.iFull > Nvm.iChrg - 20000) Nvm.iFull = Nvm.iChrg - 20000;
  if (Nvm.v1Calibration < 4000 || Nvm.v1Calibration > 40000) Nvm.v1Calibration = 40000;
  if (Nvm.v2Calibration < 800  || Nvm.v2Calibration > 1200 ) Nvm.v2Calibration = 1200;
  if (Nvm.cMax < 10  || Nvm.cMax > 10000 ) Nvm.cMax = 10;
  if (Nvm.rShunt < 100 || Nvm.rShunt > 1000) Nvm.rShunt = 1000;

  // Calculate current calibration value
  G.iCalibration = ((int32_t)I_CALIBRATION_P * 1000) / Nvm.rShunt;

  // Calculate the maximum allowed charge duration
  G.tMax = ( (2 * 3600 * Nvm.cMax) / (Nvm.iChrg / 1000) );

  // Calculate the safe charging current
  G.iSafe = Nvm.iChrg / I_SAFE_FACTOR;

}


/*
 * Read and validate EEPROM data
 */
void nvmRead (void) {
  uint32_t crc;
  
  eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm)); 
  nvmValidate ();

  // Calculate and check CRC checksum
  crc = crcCalc ((uint8_t*)&Nvm, sizeof (Nvm) - 4);
  Cli.xprintf (Str.CRC, crc);

  if (crc != Nvm.crc) G.crcOk = false;
  else G.crcOk = true;
}


/*
 * Write and validate EEPROM data
 */
void nvmWrite (void) {
  nvmValidate (); 
  Nvm.crc = crcCalc ((uint8_t*)&Nvm, sizeof (Nvm) - 4);
  Cli.xprintf (Str.CRC, Nvm.crc);
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}



/*
 * Arduino initalization routine
 */
void setup (void) {

  MCUSR = 0;      // clear MCU status register
  wdt_disable (); // and disable watchdog

  // Initialize the command-line interface
  Cli.init();
  Cli.xputs ("");
  Cli.xputs ("+ + +  L I  C H A R G E R  + + +");
  Cli.xputs ("");
  Cli.xprintf ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.xputs ("");
  Cli.newCmd ("ncells", "set N_cells", numCellsSet);
  Cli.newCmd ("cmax", "set C_max in mAh", cMaxSet);
  Cli.newCmd ("ichrg", "set I_chrg in mA", iChrgSet);
  Cli.newCmd ("ifull", "set I_full in mA", iFullSet);
  Cli.newCmd ("rshunt", "set R_shunt in mΩ", rShuntSet);
  Cli.newCmd ("c", "calibration params", showCalibration);
  Cli.newCmd (".", "real-time params", showRtParams);
  Cli.newCmd ("cal", "calibrate V1 & V2 (cal [v1|v2])", calibrate);
  
  Cli.showHelp ();

  // Initialize the ADC
  ADConv.initialize (ADC_PRESCALER_128, ADC_INTERNAL, NUM_APINS, ADC_AVG_SAMPLES);

  // Initialize digital pins
  pinMode (MOSFET_PIN, OUTPUT);

  // Initialize LED
  Led.initialize (LED_PIN);

  // Read the settngs from EEPROM and validate them
  nvmRead ();

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
  static bool iMinCalc = false;
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

  // Read the ADC channels
  adcRead ();

  // Force the error STATE if CRC error occurred
  if (!G.crcOk && G.state != STATE_ERROR) Cli.xprintf ("CRC "), G.state = STATE_ERROR_E;


  // Main state machine
  switch (G.state) {
  
    /********************************************************************/
    // Initialization State
    case STATE_INIT_E:
      Led.blink (-1, 500, 1500);
      trickleCharge = false; 
      G.vMax = (int32_t)V_MAX * Nvm.numCells; // Set vMax to full charge level
      G.vMin = (int32_t)V_MIN * Nvm.numCells;
      G.iMax = G.iSafe;
      chargeTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("Waiting for battery\n");
      G.state = STATE_INIT; 
    case STATE_INIT:
      // Start charging if vBatt stays within bounds during TIMEOUT_CHARGE
      if ( G.vBatt < G.vMin || G.vBatt > G.vMax ) chargeTs = ts;
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
      G.vMin = (int32_t)V_MIN * Nvm.numCells;
      G.iMin = Nvm.iChrg;
      iMinCalc = false;
      if (trickleCharge) Cli.xputs ("Trickle charging\n");
      else              Cli.xputs ("Charging\n");
      G.state = STATE_CHARGE;
    case STATE_CHARGE:

      // Run the regulation routine at the preset interval
      if (ts - updateTs > UPDATE_INTERVAL) {
        updateTs = ts;

        // Regulate voltage and current with the CC-CV algorithm
        if ( ( G.vBatt > G.vMax + (int32_t)V_WINDOW * Nvm.numCells ) ||
             ( G.i     > G.iMax + (int32_t)I_WINDOW ) ) {
          if (G.dutyCycle > 0) G.dutyCycle--;
          iMinCalc = true;
        }
        else if ( ( G.vBatt < G.vMax - (int32_t)V_WINDOW * Nvm.numCells ) &&
                  ( G.i     < G.iMax - (int32_t)I_WINDOW ) ) {
          if (G.dutyCycle < 255) G.dutyCycle++;
        }

        // Update the PWM duty cycle
        analogWrite (MOSFET_PIN, (int8_t)G.dutyCycle);
      }

      // Calculate the charging current
      if (G.vBatt < (uint32_t)V_SAFE * Nvm.numCells) G.iMax = G.iSafe;
      else                                           G.iMax = Nvm.iChrg;

      // Calculate the minimum allowed battery voltage
      if (G.vMin < G.vBatt - (int32_t)V_DELTA * Nvm.numCells) {
        if (G.vBatt < G.vMax) G.vMin = G.vBatt - (int32_t)V_DELTA * Nvm.numCells;
        else                  G.vMin = G.vMax  - (int32_t)V_DELTA * Nvm.numCells;
      }
      
      // Signal an error if vBatt stays out of bounds or open circuit condition occurs during TIMEOUT_ERROR
      if ( G.vBatt > (int32_t)G.vMin && 
           G.vBatt < (int32_t)V_SURGE * Nvm.numCells && 
           !(G.i == 0 && G.dutyCycle > 0)               ) errorTs = ts;
      if (ts - errorTs > TIMEOUT_ERROR) {
        showRtParams (0, NULL);
        if (G.vBatt > (int32_t)V_SURGE * Nvm.numCells) Cli.xprintf ("Overvolt ");
        if (G.vBatt < (int32_t)G.vMin )                Cli.xprintf ("Undervolt ");
        if (G.i == 0 && G.dutyCycle > 0)               Cli.xprintf ("Open circuit ");
        G.state = STATE_ERROR_E;
      }

      // Calculate historically minimum charge current
      if (iMinCalc && G.i < G.iMin) G.iMin = G.i;

      // Report battery full if iFull has not been exceeded during TIMEOUT_FULL
      if ( (G.i > Nvm.iFull || G.iMax <= Nvm.iFull) && (G.i < G.iMin + (uint32_t)I_DELTA) || G.iMax == G.iSafe ) fullTs = ts;
      if (ts - fullTs > TIMEOUT_FULL) {
        showRtParams (0, NULL);
        if (G.i < Nvm.iFull)                  Cli.xprintf("I_full"); 
        if (G.i > G.iMin + (uint32_t)I_DELTA) Cli.xprintf("I_delta");
        Cli.xputs(Str.reached);
        G.state = STATE_FULL_E; 
      }

      // Calculate charged capacity by integrating i over time
      if (ts - tickTs >= 1000 /* && !trickleCharge */) {
        tickTs += 1000;
        G.t++;
        G.c += (G.i / 1000);
      }

      // Maximum charge capacity is reached (nominal capacity + 10%, 3960 = 3600 * 1.1)
      if (G.c > (uint32_t)Nvm.cMax * 3960 && !trickleCharge) {
        showRtParams (0, NULL); 
        Cli.xputs ("C_max "); Cli.xputs(Str.reached);
        G.state = STATE_FULL_E;
      }

      // Maximum charge duration reached
      if (G.t > G.tMax && !trickleCharge) {
        showRtParams (0, NULL);
        Cli.xprintf ("T_max"); Cli.xputs(Str.reached);
        G.state = STATE_FULL_E;  
      }
      break;

    /********************************************************************/
    // Battery Full State
    case STATE_FULL_E:
      Led.blink (-1, 100, 1900);
      trickleCharge = true;
      G.vMax = (int32_t)V_TRICKLE_MAX * Nvm.numCells; // Reduce vMax to trickle charge level
      trickleTs = ts;
      resetTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("Battery full\n");
      G.state = STATE_FULL;     
    case STATE_FULL:
      // Start a trickle charging cycle if V_TRICKLE_START has not been exceeded during TIMEOUT_TRICKLE
      if (G.vBatt > (int32_t)V_TRICKLE_START * Nvm.numCells) trickleTs = ts;
      if (ts - trickleTs > TIMEOUT_TRICKLE) {
        showRtParams (0, NULL);
        G.state = STATE_CHARGE_E;
      }

      // Go to STATE_INIT if vBatt stayed 0 during TIMEOUT_RESET
      if (G.vBatt > 0) resetTs = ts;
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
      G.state = STATE_ERROR;   
    case STATE_ERROR:
      // Go to STATE_INIT if vBatt stayed 0 during TIMEOUT_RESET
      if (G.vBatt > 0) resetTs = ts;
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
  result = ADConv.readAll ();

  
  if (result) {
    // Get the ADC results
    G.v1Raw = ADConv.result[0];
    G.v2Raw = ADConv.result[1];

    // Calculate voltage and current
    G.v1 = (int32_t)G.v1Raw * Nvm.v1Calibration;
    G.v2 = (int32_t)G.v2Raw * Nvm.v2Calibration;
    G.vBatt = G.v1 - G.v2;
    G.i  = ( (int32_t)G.v2 * G.iCalibration ) / I_CALIBRATION_P ;
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
  Cli.xprintf ("I      = %lu mA\n", G.i / 1000);
  Cli.xprintf ("I_max  = %lu mA\n", G.iMax / 1000);  
  Cli.xprintf ("I_min  = %lu mA\n", G.iMin / 1000);
  Cli.xprintf ("V_batt = %lu mV\n", G.vBatt / 1000);
  Cli.xprintf ("V_max  = %lu mV\n", G.vMax / 1000);
  Cli.xprintf ("V_min  = %lu mV\n", G.vMin / 1000);
  Cli.xprintf ("PWM    = %u\n", G.dutyCycle);
  Cli.xprintf ("V1_raw = %lu\n", G.v1Raw);
  Cli.xprintf ("V2_raw = %lu\n", G.v2Raw);
  Cli.xprintf ("V1     = %lu mV\n", G.v1 / 1000);
  Cli.xprintf ("V2     = %lu mV\n", G.v2 / 1000);
  Cli.xputs("");
  return 0;
}



/*
 * CLI command for showing calibration parameters
 * These values do not change during runtime. 
 * They are either directly read from EEPROM or 
 * derived from other EEPROM values.
 */
int showCalibration (int argc, char **argv) {
  Cli.xprintf(Str.N_cells, Nvm.numCells);
  Cli.xprintf(Str.C_max,   Nvm.cMax);
  Cli.xprintf(Str.I_chrg,  Nvm.iChrg / 1000);
  Cli.xprintf(Str.I_safe,  G.iSafe / 1000);
  Cli.xprintf(Str.I_full,  Nvm.iFull / 1000);
  Cli.xprintf(Str.T_max,   G.tMax / 60);
  Cli.xprintf(Str.R_shunt, Nvm.rShunt);
  Cli.xprintf(Str.V1_cal,  Nvm.v1Calibration);
  Cli.xprintf(Str.V2_cal,  Nvm.v2Calibration);
  Cli.xprintf(Str.I_cal,   G.iCalibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the number of cells
 * argv[1]: number of cells
 */
int numCellsSet (int argc, char **argv) {
  Nvm.numCells = atol (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.N_cells, Nvm.numCells);
  Cli.xputs("");
  G.state = STATE_INIT_E;
  return 0;
}


/*
 * CLI command for setting the charge current
 * argv[1]: current in mA
 */
int iChrgSet (int argc, char **argv) {
  Nvm.iChrg = atol (argv[1]) * 1000;
  nvmWrite ();
  Cli.xprintf(Str.I_chrg, Nvm.iChrg / 1000);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the end of charge current
 * argv[1]: current in mA
 */
int iFullSet (int argc, char **argv) {
  Nvm.iFull = atol (argv[1]) * 1000;
  nvmWrite ();
  Cli.xprintf(Str.I_full, Nvm.iFull / 1000);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the battery capacity
 * argv[1]: battery capacity in mAh
 */
int cMaxSet (int argc, char **argv) {
  Nvm.cMax = atol (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.C_max, Nvm.cMax);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the shunt resistor value
 * argv[1]: shunt resistance in mΩ
 */
int rShuntSet (int argc, char **argv) {
  Nvm.rShunt = atol (argv[1]);
  nvmWrite ();
  Cli.xprintf(Str.R_shunt, Nvm.rShunt);
  Cli.xprintf(Str.I_cal, G.iCalibration);
  Cli.xputs("");
  return 0;
}


/*
 * Calibrate V1
 */
void calibrateV1 (void) {
  Nvm.v1Calibration = ((int32_t)V1_REF * Nvm.numCells) / (int32_t)G.v1Raw;
  nvmWrite ();
  Cli.xprintf(Str.V1_cal, Nvm.v1Calibration);
  Cli.xputs("");
}


/*
 * Calibrate V2
 */
void calibrateV2 (void) {
  Nvm.v2Calibration = (int32_t)V2_REF / (int32_t)G.v2Raw;
  nvmWrite ();
  Cli.xprintf(Str.V2_cal, Nvm.v2Calibration);
  Cli.xputs("");
}



/*
 * CLI command for calibrating V1 and V2
 * argv[1]:
 *   +  : increase PWM duty cycle by 1 step
 *   ++ : increase PWM duty cycle by 10 steps
 *   -  : decrease PWM duty cycle by 1 step
 *   -- : decrease PWM duty cycle by 10 steps
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
    Cli.xprintf("V1_ref = %lu mV\n", (V1_REF * Nvm.numCells) / 1000);
    Cli.xprintf("V2_ref = %lu mV\n\n", V2_REF / 1000);
  }
  return 0;
}
