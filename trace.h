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


#ifndef __TRACE_H
#define __TRACE_H

#include <Arduino.h>

/*
 * Trace message layout
 */
struct TraceMsg_t {
  uint8_t  stamp;
  char     message;
  uint16_t value;
};



/*
 * Trace class definition
 */
class TraceClass {

  public:
    /*
     * Initialize the trace object
     * Prerequisite: Serial.begin() must be called first
     * Parameters:
     *   eepromAddr : start address in EEPROM
     *   bufSize    : trace buffer size
     *   periodMs   : time duration in ms for incrementing the timestamp
     */
    void initialize (uint16_t eepromAddr, uint16_t bufSize, uint32_t periodMs);

    
    /*
     * Call this function in the Arduino main loop
     */
    void loopHandler (void);


    /*
     * Start incrementing the timestamp
     */
    void start (void);


    /*
     * Stop incrementing the timestamp
     */
    void stop (void);

    
    /*
     * Reset the timestamp, also stops incrementing
     */
    void reset (void);

    
    /*
     * Write a new trace message to EEPROM
     * Parameters:
     *   message : user-readable ascii character identifying the trace message
     *   value   : integer value
     */
    void log (char message, uint16_t value);

    /*
     * Dump the accumulated trace buffer
     */
     void dump (void);

     

  private:
    uint16_t eepromAddr = 0;
    uint16_t bufSize    = 0;
    uint16_t index      = 0;
    uint32_t periodMs   = 1;
    uint32_t stamp      = 0;
    uint32_t stampTs    = 0;
    bool     active     = false;
};







#endif // __TRACE_H
