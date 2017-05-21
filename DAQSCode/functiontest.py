
import struct, array, time, io, fcntl, datetime, signal, sys, random, csv, os, threading, spidev, datetime
from gps import *
from time import *
from time import sleep
from collections import defaultdict

from Adafruit_ADS1x15 import ADS1x15

temp = 30

boardNo='ISBBoard1'

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
    # i=1
    fileName=boardNo+'.csv'
    csv=open(fileName, 'r')
    hdr = csv.readline()
    hdr = hdr.strip().split(',')
    d = defaultdict(list)
    lines = csv.readlines()
    for line in lines:
        row = line.strip().split(',')
        for i in range(1,len(hdr)):
            key = str(hdr[i])
            value = str(row[i])
            d[key].append(value)

    csv.close()
    return d
gasSpecifics=getOffsetsAndSensitivity(boardNo)
def getConcentration(we,ae,gasName,temp):
    sensitivty=float(gasSpecifics[gasName][6])
    if ('ISB' not in boardNo):
        gasNo=gasList[gasName+'-A']
    else:
        gasNo=gasList[gasName+'-B']
    if float(temp)>50:
        temp=50
    n=float(data[gasNo-1][str(int(round(temp, -1)))])#NOTE: Will work properly at up to 50 degrees. Needs revision to work on higher temp.
    print n
    weOffset = gasSpecifics[gasName][2]
    aeOffset = gasSpecifics[gasName][5]
    print weOffset
    print aeOffset
    print sensitivty
    realValue=(float(we)-float(weOffset))-n*(float(ae)-float(aeOffset))
    realValue=realValue/sensitivty #Gives concentration in ppb
    return realValue
gasList = {'O3-A':1, 'O3-B':2, 'SO2-A':3, 'SO2-B':4, 'NO2-A':5, 'NO2-B':6, 'NO-A':7, 'NO-B':8, 'CO-A':9, 'CO-B':10, 'H2S-A':11, 'H2S-B':12}

ADS1115 = 0x01	# 16-bit ADC
adc = ADS1x15(ic=ADS1115, address=0x48)
adc2 = ADS1x15(ic=ADS1115, address=0x4A)

# Select the gain
#gain = 6144  # +/- 6.144V
#gain = 4096  # +/- 4.096V
# gain = 2048  # +/- 2.048V
# gain = 1024  # +/- 1.024V
# gain = 512   # +/- 0.512V
# gain = 256   # +/- 0.256V
gains = [6144,4096,2048,1024,512,256]
# Select the sample rate
sps = 8    # 8 samples per second
# sps = 16   # 16 samples per second
# sps = 32   # 32 samples per second
# sps = 64   # 64 samples per second
# sps = 128  # 128 samples per second
#sps = 250  # 250 samples per second
# sps = 475  # 475 samples per second
# sps = 860  # 860 samples per second

def getReadings():
                numberOfSensors=4

                volts=[]
                for i in range(2*numberOfSensors):
                        if i<=3:
                                volts.append((adc.readADCSingleEnded(i, 6144, sps)))
                                #print (volts[i])
                        else:
                                volts.append((adc2.readADCSingleEnded(i-4, 6144, sps)))
                                #print (volts[i])

                d={'SO2':[0,0],'CO':[0,0],'O3':[0,0],'NO2':[0,0]}
                d['O3'][0]=volts[0] #A0 Non-soldered
                d['O3'][1]=volts[1] #A1
                d['NO2'][0]=volts[2] #A2
                d['NO2'][1]=volts[3] #A3
                d['SO2'][0]=volts[4] #A0 Soldered
                d['SO2'][1]=volts[5] #A1
                d['CO'][0]=volts[6] #A2
                d['CO'][1]=volts[7] #A3
                return d

dictio = getReadings()

print dictio

NO2 = (getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
CO = (getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
O3 = (getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
SO2 = (getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))

print'SENSORS\n', 'NO2', NO2, '\nCO', CO, '\nO3', O3, '\nSO2', SO2
