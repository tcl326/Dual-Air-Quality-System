#!/usr/bin/python
import struct, array, time, io, fcntl, datetime

from time import *
import time
import threading
import csv

#WY Read in full data and create file to write calibrated data to.
td = open('MockData.csv', 'r')
cd = open('CalData3.csv', 'w')

#WY read in header from CSV
hdr = td.readline()
hdr = hdr.strip().split('\t')

#WY Remove the first line as this one is missing a time stamp for some reason.
td.readline()
#WY Create list (data) with each positions full data as a single entry.
data = []
#WY Store each positions full data as a dictionary in each list position (keys = headers).
for line in td:
    row = line.strip().split('\t')
    d = {}
    for i in range(len(hdr)):
        key = hdr[i]
        value = row[i]
        d[key] = value
    data.append(d)
#WY IR = 'in range' (small enough time/spatial distance to undergo calibration).
#WY Creating empty list to put data which is in range into for each sensor.
A4IR = []
B4IR = []
for i in range(len(data)):
#WY IR has been set to 10 lon/lat for now. Appropriate distance to be determined.
   if float(data[i]['Dist']) < 0.0015:
      A4IR.append(float(data[i]['A4']))
      B4IR.append(float(data[i]['B4']))
#WY Averaging the readings of each sensor for duration that they were in range of each other.
A4Ave = sum(A4IR)/len(A4IR)
B4Ave = sum(B4IR)/len(B4IR)
#WY Calculating the adjustment to shift the low grade sensor readings to be more accurate (center around high grade).
A4Shift = B4Ave-A4Ave
print('A4 Shift:', A4Shift)
#WY Write header to new calibrated data CSV
writetd = csv.DictWriter(cd, delimiter='\t', fieldnames=hdr)
writetd.writeheader()
#WY Write new data to CSV file.
for i in range(len(data)):
   cald = {}
   writetd.writerow({'Time': data[i]['Time'], 'A4': (float(data[i]['A4'])+A4Shift), 'B4': data[i]['B4'],
                              'A4Lat': data[i]['A4Lat'], 'A4Lon': data[i]['A4Lon'], 'B4Lat': data[i]['B4Lat'], 'B4Lon': data[i]['B4Lon'],
                              'Dist': data[i]['Dist']})
#Close CSV files
td.close()
cd.close()
