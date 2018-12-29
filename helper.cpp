/*
 * Helper functions 
 *
 * Karim Hraibi - 2018
 */


#include "helper.h"
#include <EEPROM.h>


#define BUTTON_LONG_PRESS_TIMEOUT (500)


/*#######################################################################################*/

void LedClass::initialize (uint8_t ledPin) {
  this->ledPin = ledPin;
  pinMode (ledPin, OUTPUT);
  digitalWrite (ledPin, LOW);
  powerOn = false;
  blinking = false;
  initialized = true;
}

void LedClass::loopHandler (void) {
  uint32_t ts;
  
  if (!initialized || !blinking ) return;

  ts = millis ();
  
  if ( (blinkOn && ts - blinkTs > tOn) || (!blinkOn && ts - blinkTs > tOff) ) {
    blinkOn = !blinkOn;
    digitalWrite (ledPin, blinkOn);
    blinkTs = ts;
    if (count > 0 ) count--;
    else if (count == 0) {
      blinkStop ();
    }
  }
}

void LedClass::turnOn (void) {
  if (!initialized) return;
  blinking = false;
  powerOn = true;
  digitalWrite (ledPin, powerOn);
}

void LedClass::turnOff (void) {
  if (!initialized) return;
  blinking = false;
  powerOn = false;
  digitalWrite (ledPin, powerOn);
}

void LedClass::toggle (void) {
  if (!initialized) return;
  blinking = false;
  powerOn = !powerOn;
  digitalWrite (ledPin, powerOn);
}

void LedClass::blink (int32_t count, uint32_t tOn, uint32_t tOff) {
  if (!initialized || blinking || count == 0) return;
  this->blinking = true;
  this->count = 2 * count;
  this->tOn = tOn;
  this->tOff = tOff;
  this->blinkOn = !powerOn;
  digitalWrite (ledPin, blinkOn);  
  blinkTs = millis ();
}

void LedClass::blinkStop (void) {
  if (!initialized || blinking ) return;
  blinking = false;
  digitalWrite (ledPin, powerOn);  
}

void LedClass::blinkBlocking (int32_t count, uint32_t tOn, uint32_t tOff) {
  blink (count, tOn, tOff);
  while (blinking) loopHandler ();
}

/*#######################################################################################*/

void ButtonClass::press (void) {
  wasPressed = pressed;
  pressed = true;
}

void ButtonClass::release (void) {
  wasPressed = pressed;
  pressed = false;
}

bool ButtonClass::rising (void) {
  bool rv = false;
  if (pressed && !wasPressed) {
    wasPressed = pressed;
    longPressTs = millis ();
    longPressed = true;
    wasLongPressed = false;
    rv = true;
  }
  return rv;
}

bool ButtonClass::falling (void) {
  bool rv = false;
  if (!pressed && wasPressed && !wasLongPressed) {
    wasPressed = pressed;
    rv = true;
  }
  return rv;  
}

bool ButtonClass::fallingLongPress (void) {
  bool rv = false;
  if (!pressed && wasPressed && wasLongPressed) {
    wasPressed = pressed;
    wasLongPressed = false;
    rv = true;
  }
  return rv;  
}

bool ButtonClass::fallingContinuous (void) {
  return !pressed && wasPressed; 
}

bool ButtonClass::longPress (void) {
  bool rv = false;
  if (pressed && longPressed && millis () - longPressTs > BUTTON_LONG_PRESS_TIMEOUT) {
    longPressed = false;
    wasLongPressed = true;
    rv = true;
  }
  return rv;
}

bool ButtonClass::longPressContinuous (void) {
  bool rv = false;
  if (pressed && millis () - longPressTs > BUTTON_LONG_PRESS_TIMEOUT) {
    longPressed = false;
    wasLongPressed = true;
    rv = true;
  }
  return rv;
}


/*#######################################################################################*/


void FirFilterClass::initialize (int16_t *memory, uint16_t size) {
  this->memory = memory;
  this->size = size;
  this->index = 0;
  this->initialized = true;
}

int16_t FirFilterClass::process (int16_t input) {
  int16_t i;
  int32_t output = 0;
  if (!initialized) return 0;
  
  memory[index] = input;
  index++;
  if (index >= size) index = 0;

  for (i = 0; i < size; i++) {
    output = output + memory[i];
  }

  return (int16_t)(output / size);
}

/*#######################################################################################*/


int16_t IirFilterClass::process (int16_t input, uint16_t size) {
  output = ((size - 1) * output + (int32_t)input) / size;
  return (int16_t)output;
}

/*#######################################################################################*/



void eepromWrite (uint16_t addr, uint8_t *buf, uint16_t bufSize) {
  uint8_t i, v;

  for (i = 0; i < bufSize && i < EEPROM.length (); i++) {
    v = EEPROM.read (addr + i);
    if ( buf[i] != v) EEPROM.write (addr + i, buf[i]);
  }
}



void eepromRead (uint16_t addr, uint8_t *buf, uint16_t bufSize) {
  uint8_t i;

  for (i = 0; i < bufSize; i++) {
    buf[i] = EEPROM.read (addr + i);
  }
}


/*#######################################################################################*/


int8_t sgn (int val) {
 if (val < 0) return -1;
 if (val == 0) return 0;
 return 1;
}
