/* 
 * Trace Buffer Implementation 
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

#include "Trace.h"
#include "Helper.h"
#include <EEPROM.h>
#include <assert.h>



void TraceClass::initialize (uint16_t eepromAddr, uint16_t bufSize, uint32_t periodMs) {

  assert (eepromAddr + sizeof(this->index) + sizeof(TraceMsg_t) * bufSize <= EEPROM.length());

  this->eepromAddr = eepromAddr;
  this->bufSize    = bufSize;
  this->periodMs   = periodMs;

  // Read the index from EEPROM
  eepromRead (eepromAddr, (uint8_t*)&this->index, sizeof(this->index));

  if (this->index > bufSize) this->index = 0;
  
}


void TraceClass::loopHandler (void) {

  if (!active) return;

  if (millis () - stampTs >= periodMs) {
    stamp++;
    stampTs += periodMs;
  }
}


void TraceClass::start (void) {
  if (active) return;
  stampTs = millis ();
  active = true;
}


void TraceClass::stop (void) {
  active = false;
}


void TraceClass::reset (void) {
  stamp = 0;
  stop ();
}
 

void TraceClass::log (char message, uint16_t value) {
  TraceMsg_t msg;

  if (bufSize == 0) return;
  
  msg.stamp   = stamp;
  msg.message = message;
  msg.value   = value;

  // Save the trace message to EEPROM
  eepromWrite (eepromAddr + sizeof(index) + index * sizeof(TraceMsg_t), (uint8_t *)&msg, sizeof(TraceMsg_t));

  index++;
  if (index >= bufSize) index = 0;

  // Save the index to EEPROM
  eepromWrite (eepromAddr, (uint8_t*)&index, sizeof(index));
}



void TraceClass::dump (void) {
  uint16_t i, idx;
  TraceMsg_t msg;

  if (bufSize == 0) return;

  // Read the index from EEPROM
  eepromRead (eepromAddr, (uint8_t*)&idx, sizeof(idx));

  for (i = 0; i < bufSize; i++) {
    // Read the trace message from EEPROM
    eepromRead (eepromAddr + sizeof(index) + idx * sizeof(TraceMsg_t), (uint8_t *)&msg, sizeof(TraceMsg_t));

    if (msg.stamp < 10)  Serial.print (' ');
    if (msg.stamp < 100) Serial.print (' ');
    Serial.print (msg.stamp, DEC);
    Serial.print (": ");
    Serial.print (msg.message);
    Serial.print (' ');
    Serial.println (msg.value, DEC);

    idx++;
    if (idx >= bufSize) idx = 0;
  }

  Serial.println();
}


 
