#
# Uninterrruptible Power Supply (UPS) for a Raspberry Pi
# Project Makefile
#
# Notes:
#
# - This script depends on the Arduino Makefile project from:
#   https://github.com/sudar/Arduino-Makefile
#
# - The following environment variables need to be defined on your
#   host system (the actual values depend on your system configuration):
#      export ARDUINO_DIR=/opt/arduino
#      export ARDMK_DIR=/opt/arduino-mk
#      export MONITOR_PORT=/dev/ttyUSB0
#
#
# This source file is part of the Raspberry Pi UPS Arduino firmware
# found under http://www.github.com/microfarad-de/pi-ups
#
# Please visit:
#   http://www.microfarad.de
#   http://www.github.com/microfarad-de
#
# Copyright (C) 2020 Karim Hraibi (khraibi at gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>. 
#




BOARD_TAG = pro
BOARD_SUB = 16MHzatmega328
MONITOR_BAUDRATE = 19200
#MONITOR_PORT = /dev/ttyUSB0 

ARDMK_DIR = arduino-mk
#ARDUINO_DIR = /Applications/Arduino.app/Contents/Java

USER_LIB_PATH = src
#ARDUINO_LIB_PATH = ../libraries
ARDUINO_LIBS = Adc Cli Led MathMf Nvm Trace EEPROM

include ${ARDMK_DIR}/Arduino.mk
