/*
 * Abstraction layer for the ATmega328p ADC 
 *
 * Karim Hraibi - 2018
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
  bitSet (ADCSRA, ADSC);                     // start a conversion 
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
