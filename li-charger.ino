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
 * Version: 1.0.0
 * Date:    January 2019
 */
#define VERSION_MAJOR 1  // major version
#define VERSION_MINOR 0  // minor version
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
#define V1_REF         4200000 // 4.20 V - Calibrate V1 against this reference voltage per cell in µV
#define V2_REF         1000000 // 1.00 V - Calibrate V2 against this reference voltage in µV
#define V_SURGE        4250000 // 4.25 V - maximum allowed surge voltage threshold per cell in µV
#define V_MAX          4190000 // 4.19 V - Maximum allowed battery voltage per cell in µV
#define V_MIN          2500000 // 2.50 V - Minimum allowed battery voltage per cell in µV
#define V_WINDOW         10000 // 0.01 V - Do not regulate voltage when within +/- this window (per cell) in µV
#define V_TICKLE_START 4050000 // 4.05 V - Tickle charge threshold voltage in µV
#define V_TICKLE_MAX   4100000 // 4.10 V - Tickle charge maximum voltage in µV
#define I_CALIBRATION_P    128 // Determines number of digits of the current calibration value (precision)
#define I_WINDOW         20000 // 0.02 A - Do not regulate current when within +/- this window in µA
#define TIMEOUT_CHARGE    2000 // Time duration in ms during which vBatt shall be between V_MIN and V_MAX before starting to charge
#define TIMEOUT_ERROR      150 // Time duration in ms during which vBatt shall be auto of bounds in order to trigger an error condition
#define TIMEOUT_FULL     10000 // Time duration in ms during which iFull shall not be exceeded in order to assume that battery is full
#define TIMEOUT_TICKLE   10000 // Time duration in ms during which vBatt shall be smaller than V_TICKLE_MAX before starting a tickle charge
#define TIMEOUT_ERR_RST   5000 // Time duration in ms during which vBatt shall be 0 before going back from STATE_ERROR to STATE_INIT
#define TIMEOUT_FULL_RST  2000 // Time duration in ms during which vBatt shall be 0 before going back from STATE_FULL to STATE_INIT
#define UPDATE_INTERVAL     50 // Interval in ms for updating the power output
#define IIR_FILTER_TAPS      1 // IIR filter taps for smoothing the ADC output
#define ADC_AVG_SAMPLES     16 // Number of ADC samples to be averaged


/* 
 * Code snippets
 */
#define EEPROM_READ() eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm)); nvmValidate ();
#define EEPROM_WRITE() nvmValidate (); eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));


/* 
 * LED object
 */
LedClass Led;


/*
 * Global variables
 */
struct {
  int32_t v1;           // V1 - Voltage at the battery '+' terminal (MOSFET drain) in µV
  int32_t v2;           // V2 - Voltage at the battery '-' terminal (shunt) in µV
  int32_t vBatt;        // Vbatt - Battery voltage = V1 - V2 in µV
  int32_t vMax;         // Maximum allowed voltage per cell during charging in µV
  int32_t i;            // I - Current in µA
  int16_t v1Raw;        // Raw ADC value of V1
  int16_t v2Raw;        // Raw ADC value of V2
  int16_t dutyCycle;    // PWM duty cycle
  int32_t tMax;         // Maximum allowed charge time duration in s
  int32_t iCalibration; // Calibration value for calculating i
  uint32_t c = 0;       // Total charged capacity in mAs
  uint32_t t = 0;       // Charge duration
  uint32_t startTs;     // Timestamp of charging begin in ms

} G;


/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  int32_t v1Calibration;  // Calibration value for calculating v1
  int32_t v2Calibration;  // Calibration value for calculating v2
  int32_t iFull;          // End of charge current in µA
  int32_t numCells;       // Number of Lithium-Ion cells
  int32_t iMax;           // Maximum charging current in µA
  int32_t rShunt;         // Shunt resistor value in mΩ
  int32_t cMax;           // Battery capacity in mAh
} Nvm;


/*
 * Validate the settings
 * Called after reading or before writing EEPROM
 * Always fall-back to the safest possible values
 */
void nvmValidate (void) {
  if (Nvm.numCells < 0 || Nvm.numCells > 6) Nvm.numCells = 1;
  if (Nvm.iMax < 100000 || Nvm.iMax > 5000000) Nvm.iMax = 100000;
  if (Nvm.iFull < 20000 || Nvm.iFull > 500000) Nvm.iFull = 500000;
  if (Nvm.v1Calibration < 4000 || Nvm.v1Calibration > 40000) Nvm.v1Calibration = 40000;
  if (Nvm.v2Calibration < 800  || Nvm.v2Calibration > 1200 ) Nvm.v2Calibration = 1200;
  if (Nvm.cMax < 10  || Nvm.cMax > 10000 ) Nvm.cMax = 10;
  if (Nvm.rShunt < 100 || Nvm.rShunt > 1000) Nvm.rShunt = 1000;

  // Calculate current calibration value
  G.iCalibration = ((int32_t)I_CALIBRATION_P * 1000) / Nvm.rShunt;

  // Calculate the maximum allowed charge duration
  G.tMax = ( (2 * 3600 * Nvm.cMax) / (Nvm.iMax / 1000) );
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
  Cli.newCmd ("imax", "set I_max in mA", iMaxSet);
  Cli.newCmd ("ifull", "set I_full in mA", iFullSet);
  Cli.newCmd ("rshunt", "set R_shunt in mΩ", rShuntSet);
  Cli.newCmd ("v1cal", "calibrate V1", v1Calibrate);
  Cli.newCmd ("v2cal", "calibrate V2", v2Calibrate);
  Cli.newCmd ("c", "calibration params", showCalibration);
  Cli.newCmd (".", "real-time params", showRtParams);
  Cli.showHelp ();

  // Initialize the ADC
  ADConv.initialize (ADC_PRESCALER_128, ADC_INTERNAL, NUM_APINS, ADC_AVG_SAMPLES);

  // Initialize digital pins
  pinMode (MOSFET_PIN, OUTPUT);

  // Initialize LED
  Led.initialize (LED_PIN);

  // Read the settngm from EEPROM and validate them
  EEPROM_READ()

  // Enable the watchdog
  wdt_enable (WDTO_1S);

}


/*
 * Arduino main loop
 */
void loop (void) {
  static enum { STATE_INIT_E, STATE_INIT, STATE_CHARGE_E, STATE_CHARGE, 
      STATE_FULL_E, STATE_FULL, STATE_ERROR_E, STATE_ERROR } state = STATE_INIT_E;
  static uint32_t tickTs = 0;
  static uint32_t updateTs = 0;
  static uint32_t chargeTs = 0;
  static uint32_t errorTs = 0;
  static uint32_t fullTs = 0;
  static uint32_t tickleTs = 0;
  static uint32_t resetTs = 0;
  static bool tickleCharge = false;
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


  // Main state machine
  switch (state) {
  
    /********************************************************************/
    // Initialization State
    case STATE_INIT_E:
      Led.blink (-1, 500, 1500);
      tickleCharge = false; 
      G.vMax = (int32_t)V_MAX; // Set vMax to full charge level
      chargeTs = ts;
      G.dutyCycle = 0;
      Cli.xputs ("Waiting for battery\n");
      state = STATE_INIT; 
    case STATE_INIT:
      // Start charging if vBatt stays within bounds during TIMEOUT_CHARGE
      if ( G.vBatt < (int32_t)V_MIN * Nvm.numCells || G.vBatt > (int32_t)V_MAX * Nvm.numCells ) chargeTs = ts;
      if (ts - chargeTs > TIMEOUT_CHARGE) {
        G.c = 0;
        G.t = 0;
        state = STATE_CHARGE_E;
      }
      break;

    /********************************************************************/
    // Charging State
    case STATE_CHARGE_E:
      Led.turnOn ();
      errorTs = ts;
      fullTs = ts;
      G.startTs = ts;
      tickTs = ts;
      Cli.xputs ("Charging\n");
      state = STATE_CHARGE;
    case STATE_CHARGE:
      // Run the regulation routine at the preset interval
      if (ts - updateTs > UPDATE_INTERVAL) {
        updateTs = ts;

        // Regulate voltage and current
        if ( ( G.vBatt > ( G.vMax + (int32_t)V_WINDOW ) * Nvm.numCells ) ||
             ( G.i     > Nvm.iMax + (int32_t)I_WINDOW ) ) {
          if (G.dutyCycle > 0) G.dutyCycle--;
        }
        else if ( ( G.vBatt < ( G.vMax - (int32_t)V_WINDOW ) * Nvm.numCells ) &&
                  ( G.i     < Nvm.iMax - (int32_t)I_WINDOW ) ) {
          if (G.dutyCycle < 255) G.dutyCycle++;
        }

        // Update the PWM duty cycle
        analogWrite (MOSFET_PIN, (int8_t)G.dutyCycle);
      }
      
      // Signal an error if vBatt stays out of bounds during TIMEOUT_ERROR
      if (G.vBatt > (int32_t)V_MIN * Nvm.numCells && G.vBatt < (int32_t)V_SURGE * Nvm.numCells) errorTs = ts;
      if (ts - errorTs > TIMEOUT_ERROR) {
        showRtParams (0, NULL);
        if (G.vBatt > (int32_t)V_SURGE * Nvm.numCells) Cli.xprintf ("Overvolt ");
        if (G.vBatt < (int32_t)V_MIN * Nvm.numCells  ) Cli.xprintf ("Undervolt ");
        state = STATE_ERROR_E;
      }

      // Report battery full if iFull has not been exceeded during TIMEOUT_FULL
      if (G.i > Nvm.iFull) fullTs = ts;
      if (ts - fullTs > TIMEOUT_FULL) Cli.xputs("I_full reached"), state = STATE_FULL_E; 

      // Calculate charged capacity by integrating i over time
      if (ts - tickTs >= 1000 && !tickleCharge) {
        tickTs = ts;
        G.t++;
        G.c += (G.i / 1000);
      }

      // Maximum charge capacity is reached (nominal capacity + 10%, 3960 = 3600 * 1.1)
      if (G.c > (uint32_t)Nvm.cMax * 3960 && !tickleCharge) Cli.xputs("C_max reached"), state = STATE_FULL_E;

      // Maximum charge duration reached
      if (G.t > G.tMax) showRtParams (0, NULL), Cli.xprintf ("Timeout "), state = STATE_ERROR_E;  
      break;

    /********************************************************************/
    // Battery Full State
    case STATE_FULL_E:
      Led.blink (-1, 100, 1900);
      tickleCharge = true;
      G.vMax = (int32_t)V_TICKLE_MAX; // Reduce vMax to tickle charge level
      tickleTs = ts;
      resetTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("Battery full\n");
      state = STATE_FULL;     
    case STATE_FULL:
      // Start a tickle charging cycle if V_TICKLE_START has not been exceeded during TIMEOUT_TICKLE
      if (G.vBatt > (int32_t)V_TICKLE_START * Nvm.numCells) tickleTs = ts;
      if (ts - tickleTs > TIMEOUT_TICKLE) state = STATE_CHARGE_E;

      // Go to STATE_INIT if vBatt stayed 0 during TIMEOUT_RESET
      if (G.vBatt > 0) resetTs = ts;
      if (ts - resetTs > TIMEOUT_FULL_RST) state = STATE_INIT_E;
      break;
        
    /********************************************************************/
    // Error State
    case STATE_ERROR_E:
      Led.blink (-1, 200, 200);
      resetTs = ts;
      G.dutyCycle = 0;
      analogWrite (MOSFET_PIN, 0);
      Cli.xputs ("ERROR\n");
      state = STATE_ERROR;   
    case STATE_ERROR:
      // Go to STATE_INIT if vBatt stayed 0 during TIMEOUT_RESET
      if (G.vBatt > 0) resetTs = ts;
      if (ts - resetTs > TIMEOUT_ERR_RST) state = STATE_INIT_E;
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
  int8_t result;
  static IirFilterClass v1Filter, v2Filter;

  // Read the ADC channels
  result = ADConv.readAll ();

  
  if (result) {
    // Smoothen results
    G.v1Raw = v1Filter.process (ADConv.result[0], IIR_FILTER_TAPS);
    G.v2Raw = v2Filter.process (ADConv.result[1], IIR_FILTER_TAPS);

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
  Cli.xprintf ("T      = %luh %lum %lus\n", hour, min, sec);
  Cli.xprintf ("C      = %lu mAh\n", G.c / 3600);
  Cli.xprintf ("V_batt = %ld mV\n", G.vBatt / 1000);
  Cli.xprintf ("V_max  = %ld mV\n", (G.vMax * Nvm.numCells) / 1000);
  Cli.xprintf ("I      = %ld mA\n", G.i / 1000);
  Cli.xprintf ("PWM    = %d\n", G.dutyCycle);
  Cli.xprintf ("V1_raw = %d\n", G.v1Raw);
  Cli.xprintf ("V2_raw = %d\n", G.v2Raw);
  Cli.xprintf ("V1     = %ld mV\n", G.v1 / 1000);
  Cli.xprintf ("V2     = %ld mV\n", G.v2 / 1000);
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
  Cli.xprintf("N_cells = %ld\n", Nvm.numCells);
  Cli.xprintf("C_max   = %ld mAh\n", Nvm.cMax);
  Cli.xprintf("I_max   = %ld mA\n", Nvm.iMax / 1000);
  Cli.xprintf("I_full  = %ld mA\n", Nvm.iFull / 1000);
  Cli.xprintf("T_max   = %ld min\n", G.tMax / 60);
  Cli.xprintf("R_shunt = %ld mΩ\n", Nvm.rShunt);
  Cli.xprintf("V1_cal  = %ld\n", Nvm.v1Calibration);
  Cli.xprintf("V2_cal  = %ld\n", Nvm.v2Calibration);
  Cli.xprintf("I_cal   = %ld\n", G.iCalibration);
  Cli.xprintf("V1_ref  = %ld mV\n", (V1_REF * Nvm.numCells) / 1000);
  Cli.xprintf("V2_ref  = %ld mV\n", V2_REF / 1000);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the number of cells
 * argv[1]: number of cells
 */
int numCellsSet (int argc, char **argv) {
  Nvm.numCells = atol (argv[1]);
  EEPROM_WRITE()
  Cli.xprintf("N_cells = %ld\n", Nvm.numCells);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the charge current
 * argv[1]: current in mA
 */
int iMaxSet (int argc, char **argv) {
  Nvm.iMax = atol (argv[1]) * 1000;
  EEPROM_WRITE()
  Cli.xprintf("I_max = %ld mA\n", Nvm.iMax / 1000);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the end of charge current
 * argv[1]: current in mA
 */
int iFullSet (int argc, char **argv) {
  Nvm.iFull = atol (argv[1]) * 1000;
  EEPROM_WRITE()
  Cli.xprintf("I_full = %ld mA\n", Nvm.iFull / 1000);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the battery capacity
 * argv[1]: battery capacity in mAh
 */
int cMaxSet (int argc, char **argv) {
  Nvm.cMax = atol (argv[1]);
  EEPROM_WRITE()
  Cli.xprintf("C_max = %ld mAh\n", Nvm.cMax);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for setting the shunt resistor value
 * argv[1]: shunt resistance in mΩ
 */
int rShuntSet (int argc, char **argv) {
  Nvm.rShunt = atol (argv[1]);
  EEPROM_WRITE()
  Cli.xprintf("R_shunt = %ld mΩ\n", Nvm.rShunt);
  Cli.xprintf("I_cal   = %ld\n", G.iCalibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for calibrating V1
 */
int v1Calibrate (int argc, char **argv) {
  Nvm.v1Calibration = ((int32_t)V1_REF * Nvm.numCells) / (int32_t)G.v1Raw;
  EEPROM_WRITE()
  Cli.xprintf("V1_cal = %ld\n", Nvm.v1Calibration);
  Cli.xputs("");
  return 0;
}


/*
 * CLI command for calibrating V2
 */
int v2Calibrate (int argc, char **argv) {
  Nvm.v2Calibration = (int32_t)V2_REF / (int32_t)G.v2Raw;
  EEPROM_WRITE()
  Cli.xprintf("V2_cal = %ld\n", Nvm.v2Calibration);
  Cli.xputs("");
  return 0;
}
