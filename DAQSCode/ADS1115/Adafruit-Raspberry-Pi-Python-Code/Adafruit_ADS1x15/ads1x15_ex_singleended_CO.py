#!/usr/bin/python

import time, signal, sys
from Adafruit_ADS1x15 import ADS1x15

def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
#print 'Press Ctrl+C to exit'

ADS1015 = 0x00  # 12-bit ADC
ADS1115 = 0x01	# 16-bit ADC

# Select the gain
#gain = 6144  # +/- 6.144V
#gain = 4096  # +/- 4.096V
# gain = 2048  # +/- 2.048V
# gain = 1024  # +/- 1.024V
# gain = 512   # +/- 0.512V
# gain = 256   # +/- 0.256V
gains = [6144,4096,2048,1024,512,256]   
# Select the sample rate
# sps = 8    # 8 samples per second
# sps = 16   # 16 samples per second
# sps = 32   # 32 samples per second
# sps = 64   # 64 samples per second
# sps = 128  # 128 samples per second
sps = 250  # 250 samples per second
# sps = 475  # 475 samples per second
# sps = 860  # 860 samples per second

# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115)

# Read channel 0 in single-ended mode using the settings above

# To read channel 3 in single-ended mode, +/- 1.024V, 860 sps use:
# volts = adc.readADCSingleEnded(3, 1024, 860)

print "6.144V Range, A0: %.6f" % (volts0)
print "6.144V Range, A1: %.6f" % (volts1)

def getReadings():
        volts0 = adc.readADCSingleEnded(0, gains[0], sps) / 1000
        volts1 = adc.readADCSingleEnded(1, gains[0], sps) / 1000
        d={'SO2':[0,0],'CO':[0,0],'O3':[0,0],'NO':[0,0]}
        for i in range(len(gains)):
                temp= adc.readADCSingleEnded(0, gains[i], sps) / 1000
                if (abs(volts0)< gains[i]/1000):
                        continue
                else:
                        volts0=temp
                        print float(gains[i])/1000," Range, A0: %.6f" % (volts0)
                        break
                


        for i in range(len(gains)):
                temp= adc.readADCSingleEnded(1, gains[i], sps) / 1000
                if (abs(volts1)< gains[i]/1000):
                        continue
                else:
                        volts1=temp
                        print float(gains[i])/1000," Range, A1: %.6f" % (volts1)
                        break
        d['CO'][0]=volts0
        d['CO'][1]=volts1
