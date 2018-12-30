# Lithium-Ion Battery Charger

This repository contains an Arduino implementation for a basic Lithium-Ion battery charger. The code has been implemented and tested on an _Arduino Pro Mini_ clone board based on the _ATmega328P_ microcontroller.

Unless stated otherwise inside the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.

## Theory of Operation

The charger does not provide any fancy balancing or battery management features. 
It simply acts as a stabilized current source as long as the battery does not reach 4.2 Volts per cell. 
Once the voltage limit has been reached, the charger would switch to the stabilized voltage mode where it would gradually reduce 
the current in order not to exceed the maximum voltage limit.

The charger ensures that a battery is connected checking its voltage. 
The charging procedure can only be initiated if a voltage between 3.6 and 4.2 Volts per cell are present at the battery terminals.

The charger detects a battery open or short circuit by detecting any sudden voltage drops or surges. 

The charging process is terminated if the current drops below 50 milliamps and stays there for more than 1 minute.

## LED Indicator

The charging status is displayed by turing on or blinking an LED as follows:

* LED truns on for half a second every 2 seconds: ready, waiting for the battery to be connected
* LED is solid on: charging
* LED turns on for 0.1 second every 2 seconds: battery fully charged
* LED blinks quickly: error

## Circuit Diagram

The charger's schematic can be found under the _/doc_ folder.
