# Lithium-Ion Battery Charger

This repository contains an Arduino implementation for a basic Lithium-Ion battery charger. The code has been implemented and tested on an _Arduino Pro Mini_ clone board based on the _ATmega328P_ microcontroller.

Unless stated otherwise in the source file headers, please feel free to use and distribute this code under the *GNU General Public License v3.0*.

*Disclaimer: overcharging, short-circuiting or otherwise abusing Lithium-Ion batteries may result in a fire and/or a violent explosion. I neither take any responsiblity nor can be held liable for any damage caused to human beings and things due to the improper handling of lithium-Ion batteries. Please implement this project at your own risk!*

## Theory of Operation

The charger does not provide any fancy balancing or battery management features. 
Charging is accomplished with the Constant Current Constant Voltage (CC-CV) scheme. The charger acts as a stabilized source as long as the battery does not reach 4.2 Volts per cell. 
Once the voltage limit has been reached, the charger would switch to the stabilized voltage mode where it would gradually reduce 
the current as not to exceed the maximum voltage limit.

In order to avoid damaging the Lithium-Ion cells due to the charger misbehaving, it is highly recommended to additionally protect the cells by means of an off-the-shelf Battery Management System (BMS) board.

The charger ensures that a Lithium-Ion battery is connected checking its voltage. 
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

The charger's schematic can be found under the */doc* folder or can be downloaded using the follwoing link:
https://github.com/microfarad-de/li-charger/raw/master/doc/li-charger-schematic.png
