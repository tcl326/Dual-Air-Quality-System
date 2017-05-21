#!/bin/bash

killall gpsd
gpsd -S9999 /dev/ttyUSB0

# RUN PYTHON SCRIPT
#python2 YourCode.py
