#!/usr/bin/python
import struct, array, time, io, fcntl, datetime
import os
from gps import *
from time import *
import time
import threading
import csv
from random import randint
td = open('MockData.csv', 'w')
gpsd = None #seting the global variable

I2C_SLAVE=0x0703
HTU21D_ADDR = 0x40
CMD_READ_TEMP_HOLD = "\xE3"
CMD_READ_HUM_HOLD = "\xE5"
CMD_READ_TEMP_NOHOLD = "\xF3"
CMD_READ_HUM_NOHOLD = "\xF5"
CMD_WRITE_USER_REG = "\xE6"
CMD_READ_USER_REG = "\xE7"
CMD_SOFT_RESET= "\xFE"

class i2c(object):
   def __init__(self, device, bus):

      self.fr = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
      self.fw = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)

      # set device address

      fcntl.ioctl(self.fr, I2C_SLAVE, device)
      fcntl.ioctl(self.fw, I2C_SLAVE, device)

   def write(self, bytes):
      self.fw.write(bytes)

   def read(self, bytes):
      return self.fr.read(bytes)

   def close(self):
      self.fw.close()
      self.fr.close()

class HTU21D(object):
   def __init__(self):
      self.dev = i2c(HTU21D_ADDR, 1) #HTU21D 0x40, bus 1
      self.dev.write(CMD_SOFT_RESET) #soft reset
      time.sleep(.1)

   def ctemp(self, sensorTemp):
      tSensorTemp = sensorTemp / 65536.0
      return -46.85 + (175.72 * tSensorTemp)

   def chumid(self, sensorHumid):
      tSensorHumid = sensorHumid / 65536.0
      return -6.0 + (125.0 * tSensorHumid)

   def crc8check(self, value):
      # Ported from Sparkfun Arduino HTU21D Library: https://github.com/sparkfun/HTU21D_Breakout
      remainder = ( ( value[0] << 8 ) + value[1] ) << 8
      remainder |= value[2]
      
      # POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1
      # divsor = 0x988000 is the 0x0131 polynomial shifted to farthest left of three bytes
      divsor = 0x988000
      
      for i in range(0, 16):
         if( remainder & 1 << (23 - i) ):
            remainder ^= divsor
         divsor = divsor >> 1
      
      if remainder == 0:
         return True
      else:
         return False
   
   def read_tmperature(self):
      self.dev.write(CMD_READ_TEMP_NOHOLD) #measure temp
      time.sleep(.1)

      data = self.dev.read(3)
      buf = array.array('B', data)

      if self.crc8check(buf):
         temp = (buf[0] << 8 | buf [1]) & 0xFFFC
         return self.ctemp(temp)
      else:
         return -255
         
   def read_humidity(self):
      self.dev.write(CMD_READ_HUM_NOHOLD) #measure humidity
      time.sleep(.1)

      data = self.dev.read(3)
      buf = array.array('B', data)
      
      if self.crc8check(buf):
         humid = (buf[0] << 8 | buf [1]) & 0xFFFC
         return self.chumid(humid)
      else:
         return -255

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps("localhost", "9999")
    gpsd.stream(WATCH_ENABLE) #starting the stream of info
#    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

header = ['Time', 'A4', 'B4', 'A4Lat', 'A4Lon', 'B4Lat', 'B4Lon', 'Dist'] #, 'Altitude', 'Speed', 'Climb', 'Heading', 'EPS', 'EPX', 'EPV', 'EPT']
writetd = csv.DictWriter(td, delimiter='\t', fieldnames=header)
writetd.writeheader()
k = 40

if __name__ == "__main__":
   obj = HTU21D()
   B4 = 610+round(obj.read_tmperature(),4)
   gpsp = GpsPoller() # create the thread
   try:
      gpsp.start() # start it up
      while True: #int(gpsd.fix.mode) != 1: #mode: 0 = no mode, 1 = no fix, 2 = 2D fix, 3 = 3D fix
         #It may take a second or two to get good data
         #os.system('clear')
         #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
         time.sleep(0.1)
         #print "Temp:", round(obj.read_tmperature(),4), "C"
         #print "Humidity:", round(obj.read_humidity(),4), "% rH"
         #print 'Latitude:' , gpsd.fix.latitude
         #print 'Longitude:' , gpsd.fix.longitude
         Time = (gpsd.utc) #datetime.datetime.now()) #Full date and time
         A4 = (round(obj.read_tmperature(),4)+600+randint(-1,1)) #Using temp sensor to make mock OXA sensor data
         B4 += (randint(-1,2)) #Using temp sensor to make mock OXA sensor data
         A4Lat = (gpsd.fix.latitude) #A4Latitude (Decimal)
         A4Lon = (gpsd.fix.longitude) #A4longitude (Decimal)
         B4Lat = (gpsd.fix.latitude+k) #B4Latitude (Decimal)
         B4Lon = (gpsd.fix.longitude+k) #B4longitude (Decimal)
         Dist = (((B4Lon-A4Lon)**2+(B4Lat-A4Lat)**2)**0.5)
         Alt = (gpsd.fix.altitude)#altitude (meters)
         EPS = (gpsd.fix.eps) #speed error (meters/sec)
         EPX = (gpsd.fix.epx) #longitude error
         EPV = (gpsd.fix.epv) #altitude error (meters)
         EPT = (gpsd.fix.ept) #time error
         Speed = (gpsd.fix.speed) #speed (meters/sec)
         Climb = (gpsd.fix.climb) #change in elev
         Head = (gpsd.fix.track) #heading
         if int(gpsd.fix.mode) != 1 and int(gpsd.fix.mode) != 0:
            print 'Level', gpsd.fix.mode, 'Sat Fix\nNow writing data...'
            writetd.writerow({'Time': Time, 'A4': A4, 'B4': B4,
                              'A4Lat': A4Lat, 'A4Lon': A4Lon, 'B4Lat': B4Lat, 'B4Lon': B4Lon,
                              'Dist': Dist})
            k -= 1
                              #'Altitude': Alt, 'Speed': Speed, 'Climb': Climb, 'Heading': Head,
                              #'EPS': EPS, 'EPX': EPX, 'EPV': EPV, 'EPT': EPT})
         else:
            print'Waiting for GPS fix'
   except (KeyboardInterrupt, SystemExit): #End program when you press ctrl+c
      print ("\nEnding Thread...")
      gpsp.running = False
      gpsp.join() # wait for the thread to finish what it's doing
      print ("Done.")
td.close()
   
#Check for connectivity. If connected, up to database, else, store locally
#Check stability, boot sequence, GPS
#Plug in to waqi.info for international
