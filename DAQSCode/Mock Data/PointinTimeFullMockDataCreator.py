#!/usr/bin/python
import  datetime
import os
from time import *
import time
import csv
from random import randint
td = open('PointinTimeFullMockData.csv', 'w')

header = ['Time', 'OXA', 'OXB', 'SO2A', 'SO2B', 'NO2A', 'NO2B', 'NOA', 'NOB', 'COA', 'COB', 'H2SA', 'H2SB', 'PM10']
          #'Lat', 'Lon']# 'Altitude', 'Speed', 'Climb', 'Heading', 'EPS', 'EPX', 'EPV', 'EPT'] #'B4Lat', 'B4Lon', 'Dist'] 
writetd = csv.DictWriter(td, delimiter='\t', fieldnames=header)
writetd.writeheader()
##k = 40

##if __name__ == "__main__":
  # obj = HTU21D()
   #gpsp = GpsPoller() # create the thread
##   try:
##      gpsp.start() # start it up
##      while True: #int(gpsd.fix.mode) != 1: #mode: 0 = no mode, 1 = no fix, 2 = 2D fix, 3 = 3D fix
##         #It may take a second or two to get good data
##         #os.system('clear')
##         #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
##         time.sleep(0.1)
##         #print "Temp:", round(obj.read_tmperature(),4), "C"
         #print "Humidity:", round(obj.read_humidity(),4), "% rH"
         #print 'Latitude:' , gpsd.fix.latitude
         #print 'Longitude:' , gpsd.fix.longitude
Time = 0000#(gpsd.utc) #datetime.datetime.now()) #Full date and time
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
#Lat = [24.486289, 24.490976, 24.488867, 24.494569, 24.47965] #A4Latitude (Decimal)
#Lon = [54.353442, 54.360223, 54.362369, 54.370694, 54.355931] #A4longitude (Decimal)
         #Dist = (((B4Lon-A4Lon)**2+(B4Lat-A4Lat)**2)**0.5)
##         Alt = (gpsd.fix.altitude)#altitude (meters)
##         EPS = (gpsd.fix.eps) #speed error (meters/sec)
##         EPX = (gpsd.fix.epx) #longitude error
##         EPV = (gpsd.fix.epv) #altitude error (meters)
##         EPT = (gpsd.fix.ept) #time error
##         Speed = (gpsd.fix.speed) #speed (meters/sec)
##         Climb = (gpsd.fix.climb) #change in elev
##         Head = (gpsd.fix.track) #heading
  #       if int(gpsd.fix.mode) != 1 and int(gpsd.fix.mode) != 0:
           # print ('Level', gpsd.fix.mode, 'Sat Fix\nNow writing data...')
for i in range(300):
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
            #'Altitude': Alt, 'Speed': Speed, 'Climb': Climb, 'Heading': Head,
             #                 'EPS': EPS, 'EPX': EPX, 'EPV': EPV, 'EPT': EPT})
                              #'A4': A4, 'B4': B4, 'B4Lat': B4Lat, 'B4Lon': B4Lon, 'Dist': Dist})
  #       else:
  #          print('Waiting for GPS fix')
##   except (KeyboardInterrupt, SystemExit): #End program when you press ctrl+c
##      print ("\nEnding Thread...")
##      gpsp.running = False
##      gpsp.join() # wait for the thread to finish what it's doing
##      print ("Done.")
td.close()
   
#Check for connectivity. If connected, up to database, else, store locally
#Check stability, boot sequence, GPS
#Plug in to waqi.info for international
