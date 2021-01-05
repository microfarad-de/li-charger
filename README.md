# Lithium-Ion Battery Charger

This repository contains an Arduino implementation for a basic Lithium-Ion battery charger. The code has been implemented and tested on an _Arduino Pro Mini_ clone board based on the _ATmega328P_ microcontroller.

This project uses Git submodules. In order to get its full source code,please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

Please visit http://www.microfarad.de/li-charger for a full description of this project.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the *GNU General Public License v3.0*.

*Disclaimer: overcharging, short-circuiting or otherwise abusing Lithium-Ion batteries may result in a fire and/or a violent explosion. The author of this code neither takes any responsibility nor can be held liable for any damage caused to human beings and things due to the improper handling of Lithium-Ion batteries. Please implement this project at your own risk!*

## Theory of Operation

Charging is accomplished with the Constant Current Constant Voltage (CC-CV) scheme. The charger acts as a stabilized source as long as the battery does not reach 4.2 Volts per cell. 
Once the voltage limit has been reached, the charger would switch to the stabilized voltage mode where it would gradually reduce 
the current as not to exceed the maximum voltage limit.

In order to avoid damaging the Lithium-Ion cells due to the charger misbehaving, it is highly recommended to additionally protect the cells by means of an off-the-shelf Battery Management System (BMS) board.

The charger detects a battery open or short circuit by detecting any sudden voltage drops or surges. 

The charging process is terminated if the current drops below a certain threshold (percentage of the battery capacity) and stays there for a pre-determined amount of time.

## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Circuit Diagram

The charger's schematic can be found under the */doc* folder or can be downloaded using the follwoing link:
https://github.com/microfarad-de/li-charger/raw/master/doc/li-charger-schematic.png

