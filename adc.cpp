/* 
 * Abstraction layer for the ATmega328p ADC 
 * Non-blocking read the ADC output as an alternative
 * to the blocking analogRead() method.
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
 */

#include "adc.h"
#include <assert.h>


AdcClass ADConv;


void AdcClass::initialize (AdcPrescaler_t prescaler, AdcReference_t reference, uint8_t numPins) {
  this->reference = reference;
  assert (numPins < ADC_NUM_PINS);
  this->numPins = numPins;
  ADCSRA =  _BV (ADEN);   // turn ADC on
  ADCSRA |= prescaler ;
}


void AdcClass::start (AdcPin_t adcPin) {
  uint8_t pin;
  
  if (working) return;
  
  pin = (uint8_t)adcPin;
  ADMUX  = reference | (pin & 0x07);      // select reference and input port
  bitSet (ADCSRA, ADSC);                  // start a conversion 
  working = true;
}


int16_t AdcClass::readVal (void) {
  int16_t rv;
  
  // the ADC clears the bit when done
  if (bit_is_clear(ADCSRA, ADSC) && working) {
    rv = ADC; // read result
    working = false;
    return rv;
  }
  else {
    return -1;
  }
}


int8_t AdcClass::readAll (void) {
  int16_t adcVal;
  uint8_t pin = currentPin;

  // Start a new ADC measurement
  start ((AdcPin_t)currentPin);
  
  // Check if ADC finished detecting the value
  adcVal = readVal ();

  // ADC finished
  if (adcVal >= 0) {
    result[currentPin] = adcVal;
    currentPin++;
    if (currentPin >= numPins) currentPin = 0; 
    return pin;
  }
  else {
    return -1;
  }   
}
