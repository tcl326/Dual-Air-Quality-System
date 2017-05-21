#!/usr/bin/python
import  datetime
import os
from time import *
import time
import csv
from random import randint
td = open('PointinTimeFullMockData.csv', 'w')

header = ['Time', 'OXA', 'OXB', 'SO2A', 'SO2B', 'NO2A', 'NO2B', 'NOA', 'NOB', 'COA', 'COB', 'H2SA', 'H2SB', 'PM10']
writetd = csv.DictWriter(td, delimiter='\t', fieldnames=header)
writetd.writeheader()

Time = 0#(gpsd.utc) #datetime.datetime.now()) #Full date and time
OXA = (40+randint(-1,1)) #Using temp sensor to make mock AIR sensor data (20-100 ug/m^3)
OXB = 40+randint(-1,2) #Using temp sensor to make mock AIR sensor data (20-100 ug/m^3)
SO2A = randint(-1,1) #Using temp sensor to make mock AIR sensor data (0-10 ug/m^3)
SO2B = randint(-1,2) #Using temp sensor to make mock AIR sensor data (0-10 ug/m^3)
NO2A = 10+randint(-1,1) #Using temp sensor to make mock AIR sensor data (5-50 ug/m^3)
NO2B = 10+randint(-1,2) #Using temp sensor to make mock AIR sensor data (5-50 ug/m^3)
NOA =  randint(-1,1) #Using temp sensor to make mock AIR sensor data 
NOB = randint(-1,2) #Using temp sensor to make mock AIR sensor data
COA = 0 #(round(obj.read_tmperature(),4)+600+randint(-1,1)) #Using temp sensor to make mock AIR sensor data (0 ug/m^3)
COB = 0 #(round(obj.read_tmperature(),4)+600+randint(-1,2)) #Using temp sensor to make mock AIR sensor data (0 ug/m^3)
H2SA = 5+randint(-1,1) #Using temp sensor to make mock AIR sensor data
H2SB = 5+randint(-1,2) #Using temp sensor to make mock AIR sensor data
PM10 = 7+randint(-1,2) #Using temp sensor to make mock AIR sensor data (5-30 ug/m^3)

#Brooke: Please place each mock data entry at the coordinates below
#for i in range(5):
#   'Lat': Lat[i], 'Lon': Lon[i]
#Lat = [24.486289, 24.490976, 24.488867, 24.494569, 24.47965] #A4Latitude (Decimal)
#Lon = [54.353442, 54.360223, 54.362369, 54.370694, 54.355931] #A4longitude (Decimal)
         #Dist = (((B4Lon-A4Lon)**2+(B4Lat-A4Lat)**2)**0.5)

for i in range(1440):
   writetd.writerow({'Time': Time,  'OXA': abs(OXA), 'OXB': abs(OXB), 'SO2A': abs(SO2A), 'SO2B': abs(SO2B), 'NO2A': abs(NO2A), 'NO2B': abs(NO2B), 'NOA': abs(NOA), 'NOB': abs(NOB), 'COA': abs(COA), 'COB': abs(COB), 'H2SA': abs(H2SA), 'H2SB': abs(H2SB), 'PM10': abs(PM10)})#'Lat': Lat[i], 'Lon': Lon[i]})
   Time += 1#(gpsd.utc) #datetime.datetime.now()) #Full date and time
   OXA += randint(-1,1) #Using temp sensor to make mock AIR sensor data (20-100 ug/m^3)
   OXB += randint(-1,2) #Using temp sensor to make mock AIR sensor data (20-100 ug/m^3)
   SO2A += randint(-1,1) #Using temp sensor to make mock AIR sensor data (0-10 ug/m^3)
   SO2B += randint(-1,2) #Using temp sensor to make mock AIR sensor data (0-10 ug/m^3)
   NO2A += randint(-1,1) #Using temp sensor to make mock AIR sensor data (5-50 ug/m^3)
   NO2B += randint(-1,2) #Using temp sensor to make mock AIR sensor data (5-50 ug/m^3)
   NOA +=  randint(-1,1) #Using temp sensor to make mock AIR sensor data 
   NOB += randint(-1,2) #Using temp sensor to make mock AIR sensor data
   COA = 0 #(round(obj.read_tmperature(),4)+600+randint(-1,1)) #Using temp sensor to make mock AIR sensor data (0 ug/m^3)
   COB = 0 #(round(obj.read_tmperature(),4)+600+randint(-1,2)) #Using temp sensor to make mock AIR sensor data (0 ug/m^3)
   H2SA += randint(-1,1) #Using temp sensor to make mock AIR sensor data
   H2SB += randint(-1,2) #Using temp sensor to make mock AIR sensor data
   PM10 += randint(-1,2)

print('File "PointinTimeFullMockData.csv" created.')
td.close()
