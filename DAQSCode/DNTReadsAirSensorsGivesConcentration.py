#!/usr/bin/python

import struct, array, time, io, fcntl, datetime, signal, sys, random, csv, os, threading, spidev
from gps import *
from time import *
from opc import OPCN2
from time import sleep
from Adafruit_ADS1x15 import ADS1x15
from collections import defaultdict
##import time, signal, sys
##from Adafruit_ADS1x15 import ADS1x15

#Defines the number of the AFE/ISB board
boardNo='25-000009'

#WY Read in header of CSV file.
sq = open('tddc.csv', mode='r')
hdr = sq.readline()
hdr = hdr.strip().split(',')
#WY Create list (data) with each positions full data (temp and coefficient) as a single entry.
data = []
#WY Store each positions full data as a dictionary in each list position (keys = temp, values = coeff)
for line in sq:
    row = line.strip().split(',')
    d = {}
    for i in range(len(hdr)):
        key = hdr[i]
        value = row[i]
        d[key] = value
    data.append(d)
sq.close()

#A function that opens a csv and retrieves a dictionary
def getOffsetsAndSensitivity(boardNo):
    i=1
    fileName=boardNo+'.csv'
    csv=open(fileName, 'r')
    hdr = csv.readline()
    hdr = hdr.strip().split(',')
    d = defaultdict(list)
    for line in csv:
        row = line.strip().split(',')
        for i in range(len(hdr)):
            key = str(hdr[i])
            value = str(row[i])
            d[key].append(value)
            
    csv.close()    
    return d
gasSpecifics=getOffsetsAndSensitivity(boardNo)
def getConcentration(we,ae,gasName,temp):
    sensitivty=int(gasSpecifics[gasName][6])
    if (boardNo!='ISB'):
        gasNo=gasList[gasName+'-A']
    else:
        gasNo=gasList[gasName+'-B']
    if float(temp)>50:
        temp=50
    n=float(data[gasNo-1][str(int(round(temp, -1)))])#NOTE: Will work properly at up to 50 degrees. Needs revision to work on higher temp.
    realValue=(float(we)-float(gasSpecifics[gasName][2]))-n*(float(ae)-float(gasSpecifics[gasName][5]))
    realValue=realValue/sensitivty #Gives concentration in ppb
    return realValue
gasList = {'O3-A':1, 'O3-B':2, 'SO2-A':3, 'SO2-B':4, 'NO2-A':5, 'NO2-B':6, 'NO-A':7, 'NO-B':8, 'CO-A':9, 'CO-B':10, 'H2S-A':11, 'H2S-B':12}

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


while True:
        def getReadings():
                numberOfSensors=4

                volts=[]
                for i in range(2*numberOfSensors):
                        if i<=3:
                                volts.append(adc.readADCSingleEnded(i, gains[0], sps) / 1000)
                                #print (volts[i])
                        else:
                                volts.append(adc2.readADCSingleEnded(i-4, gains[0], sps) / 1000)
                                #print (volts[i])
                                
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
        #print (dictio)
        sleep (1)
        for i in dictio:
                we = (dictio[i][0])
                ae = (dictio[i][1])
                gasName = i
                temp=23#int(input('temp: '))
                print i, abs(getConcentration(we,ae,gasName,temp))   



