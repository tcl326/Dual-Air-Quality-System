#!/usr/bin/python

import time, signal, sys
from Adafruit_ADS1x15 import ADS1x15

def signal_handler(signal, frame):
        print ('You pressed Ctrl+C!')
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
#sps = 250  # 250 samples per second
# sps = 475  # 475 samples per second
sps = 860  # 860 samples per second

# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115, address=0x48)
adc2=ADS1x15(ic=ADS1115, address=0x4A)

# Read channel 0 in single-ended mode using the settings above

# To read channel 3 in single-ended mode, +/- 1.024V, 860 sps use:
# volts = adc.readADCSingleEnded(3, 1024, 860)


while 1:
        def getReadings():
                numberOfSensors=4

                volts=[]
                for i in range(2*numberOfSensors):
                        if i<=3:
                                volts.append(adc.readADCSingleEnded(i, gains[0], sps) / 1000)
                                print (volts[i])
                        else:
                                volts.append(adc2.readADCSingleEnded(i-4, gains[0], sps) / 1000)
                                print (volts[i])
                                
                d={'SO2':[0,0],'CO':[0,0],'O3':[0,0],'NO2':[0,0]}
                for j in range(2*numberOfSensors):
                        for i in range(len(gains)):
                                if (abs(float(volts[j]))< float(gains[i])/1000):
                                        if j<=3:
                                                temp= adc.readADCSingleEnded(j, gains[i], sps) / 1000
                                        else:
                                                temp= adc2.readADCSingleEnded(j-4, gains[i], sps) / 1000 
                                        continue
                                else:
                                        volts[j]=temp
                                        break

                d['CO'][0]=volts[0] #A0 Non-soldered
                d['CO'][1]=volts[1] #A1
                d['O3'][0]=volts[2] #A2
                d['O3'][1]=volts[3] #A3
                d['SO2'][0]=volts[4] #A0 Soldered
                d['SO2'][1]=volts[5] #A1
                d['NO2'][0]=volts[6] #A2
                d['NO2'][1]=volts[7] #A3             
                return d
        dictio=getReadings()
        print (dictio)
        time.sleep (2)


