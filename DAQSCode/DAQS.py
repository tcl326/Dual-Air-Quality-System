#Dual Air Quality System
#Comprehensive Script for: -saving air quality data to CSV
#                          -plotting data in real time
#                          -uploading data to Firebase in real time
#                          -converting data from concentration to AQI format
#
#Select preferences below:
usegpstime = True
includeParticleSensor = False
plotRealTime = True
saveToCSV = False
uploadToFirebase = False
convertToAQI = False

started=False
firstCycle=True
interval= 2

import struct, array, time, io, fcntl, datetime, signal, sys, random, csv, os, threading, spidev
import aqi
from gps import *
from time import *
from opc import OPCN2
import spidev
from time import sleep
import matplotlib.pyplot as plt
from Adafruit_ADS1x15 import ADS1x15
from collections import defaultdict
from Subfact_ina219 import INA219
import sys
import json
import csv 
import urllib2
import time
import calendar
import os
os.chdir("/home/pi/Desktop/Code/")
url = 'https://air-quality-readings.firebaseio.com/readings.json'

#Instantiating Current Sensor
#Battery
ina = INA219(address=0x41)
result = ina.getBusVoltage_V()
#Solar Panel
ina2 = INA219(address=0x44)
result = ina2.getBusVoltage_V()

gpsd = None #seting the global variable
gpsfixed = False
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
      sleep(0.1)

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
      sleep(0.1)
      data = self.dev.read(3)
      buf = array.array('B', data)

      if self.crc8check(buf):
         temp = (buf[0] << 8 | buf [1]) & 0xFFFC
         return self.ctemp(temp)
      else:
         return -255
         
   def read_humidity(self):
      self.dev.write(CMD_READ_HUM_NOHOLD) #measure humidity
      sleep(0.1)
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


#Defines the number of the AFE/ISB board
boardNo='25-000010'

#Read in header of temp dependent factor.
sq = open('tddc.csv', mode='r')
hdr = sq.readline()
hdr = hdr.strip().split(',')
#Create list (data) with each positions full data (temp and coefficient) as a single entry.
data = []
#Store each positions full data as a dictionary in each list position (keys = temp, values = coeff)
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
        print ('Program Terminated')
        gpsp.running = False
        gpsp.join()
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


#First Time CSV Setup
if saveToCSV == True:
  td = open('DAQSdata.csv', 'a')
  header = ['Time', 'Temp', 'CO', 'O3', 'SO2', 'NO2', 'PM1', 'PM2.5', 'PM10', 'Humidity', 'Latitude', 'Longitude', 'Altitude', 'Speed', 'Climb', 'Heading', 'EPS', 'EPX', 'EPV', 'EPT', 'BATV', 'BATI', 'BATP', 'SOLV', 'SOLI', 'SOLP']
  writetd = csv.DictWriter(td, delimiter='\t', fieldnames=header)
  writetd.writeheader()
  td.close()

def saveToCSV():
  td = open('DAQSdata.csv', 'a')
  header = ['Time', 'Temp', 'CO', 'O3', 'SO2', 'NO2', 'PM1', 'PM2.5', 'PM10', 'Humidity', 'Latitude', 'Longitude', 'Altitude', 'Speed', 'Climb', 'Heading', 'EPS', 'EPX', 'EPV', 'EPT', 'BATV', 'BATI', 'BATP', 'SOLV', 'SOLI', 'SOLP']
  writetd = csv.DictWriter(td, delimiter='\t', fieldnames=header)
  writetd.writerow({'Time': Time, 'CO': str(co), 'O3': str(o3), 'SO2': str(so2), 'NO2': str(no2), 
                             'PM1': pm1, 'PM2.5': pm2, 'PM10': pm10,
                             'BATV':BATV, 'BATI': BATI, 'BATP': BATP, 'SOLV': SOLV, 'SOLI': SOLI, 'SOLP': SOLP,
                             'Temp': temp, 'Humidity': Humidity,
                             'Latitude': Lat, 'Longitude': Lon,
                             'Altitude': Alt, 'Speed': Speed, 'Climb': Climb, 'Heading': Head,
                             'EPS': EPS, 'EPX': EPX, 'EPV': EPV, 'EPT': EPT})
  td.close()

def includeParticleSensor():
  spi=spidev.SpiDev()
  spi.open(0,0)
  spi.mode=1
  spi.max_speed_hz=500000
  alphasense=OPCN2(spi)
  alphasense.on()

def Firebase():
  postdata = {'Time': Time, 'CO': str(co), 'O3': str(o3), 'SO2': str(so2), 'NO2': str(no2),
                             'BATV':BATV, 'BATI': BATI, 'BATP': BATP, 'SOLV': SOLV, 'SOLI': SOLI, 'SOLP': SOLP,
                             'Temp': Temp, 'Humidity': Humidity,
                             'Latitude': Lat, 'Longitude': Lon,
                             'Altitude': Alt, 'Speed': Speed, 'Climb': Climb, 'Heading': Head,
                             'EPS': EPS, 'EPX': EPX, 'EPV': EPV, 'EPT': EPT}
  req = urllib2.Request(url)
  req.add_header('Content-Type', 'application/json')
  firebasedata = json.dumps(postdata)

  print('Sending...')
  try:
    if firebasedata != '':
      response = urllib2.urlopen(req,firebasedata)
    else:
      print('No Data received')
  except httplib.BadStatusLine:
    pass

def AQI():
  no2 = aqi.to_aqi([(aqi.POLLUTANT_no2_1h, str(no2))]) #ppb
  co = aqi.to_aqi([(aqi.POLLUTANT_co_8h, str(co))]) #ppm
  o3 = aqi.to_aqi([(aqi.POLLUTANT_o3_1h, str(o3))]) #ppm, 8hr
  so2 = aqi.to_aqi([(aqi.POLLUTANT_so2_1h, str(so2))]) #ppb
  pm1 = aqi.to_aqi([(aqi.POLLUTANT_PM1, str(pm1))])
  pm2 = aqi.to_aqi([(aqi.POLLUTANT_PM25, str(pm2))]) #ug/m^3
  pm10 = aqi.to_aqi([(aqi.POLLUTANT_PM10, str(pm10))]) #ug/m^3

#Set up empty graph to chart on
   
def Plot():
    
    plt.clf()
    plt.ax = plt.subplot(321)
    plt.ax2 = plt.subplot(322, sharex = ax)
    plt.ax3 = plt.subplot(323, sharex = ax)
    plt.ax4 = plt.subplot(324, sharex = ax)
    plt.ax5 = plt.subplot(325, sharex = ax)
    plt.ax6 = plt.subplot(326, sharex = ax)
    
    plt.ax.grid(True)
    plt.ax2.grid(True)
    plt.ax3.grid(True)
    plt.ax4.grid(True)
    plt.ax5.grid(True)
    plt.ax6.grid(True)

    plt.ax.set(ylabel='Deg C', title='DAQS')
    plt.ax2.set(ylabel='%H', title='DAQS')
    plt.ax3.set(ylabel='PPB')
    plt.ax4.set(ylabel='PPB')
    plt.ax5.set(ylabel='PPB')
    plt.ax6.set(ylabel='PPB')

    plt.ax.plot(tempC, 'rx-', label='Temp')
    plt.ax2.plot(Humid, 'cx-', label='Humidity')
    plt.ax4.plot(GNO2, 'bx-', label='NO2')
    plt.ax3.plot(GCO, 'gx-', label='CO')
    plt.ax5.plot(GO3, 'mx-', label='O3')
    plt.ax6.plot(GSO2, 'kx-', label='SO2')
    
    plt.ax.legend(loc='upper left')
    plt.ax2.legend(loc='upper left')
    plt.ax3.legend(loc='upper left')
    plt.ax4.legend(loc='upper left')
    plt.ax5.legend(loc='upper left')
    plt.ax6.legend(loc='upper left')

    #plt.plot(tempC, Humid, GNO2, GCO, GO3, GSO2)
    plt.show()



#INSTANTIATING EVERYTHING
obj = HTU21D()
gpsp = GpsPoller()
gpsp.start()

#Time = str(gpsd.utc)
Time1=None
sleep (0.1)
def getReadings():
       numberOfSensors=4

       volts=[]
       for i in range(2*numberOfSensors):
               if i<=3:
                       volts.append(adc.readADCSingleEnded(i, gains[0], sps) / 1000)
               else:
                       volts.append(adc2.readADCSingleEnded(i-4, gains[0], sps) / 1000)
                       
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
NO2 = []
CO =[]
O3 =[]
SO2 =[]
PM =[]
PM1=[]
PM2=[]
PM10=[]      

count = 0
try:
   while True:
           dictio=getReadings()
           #print (dictio)
           sleep (1)
           if gpsfixed:
                   Time = str(datetime.datetime.now())
                   Lat = str(gpsd.fix.latitude) #Latitude (Decimal)
                   Lon = str(gpsd.fix.longitude) #longitude (Decimal)
                   Alt = str(gpsd.fix.altitude)#altitude (meters)
                   EPS = str(gpsd.fix.eps) #speed error (meters/sec)
                   EPX = str(gpsd.fix.epx) #longitude error
                   EPV = str(gpsd.fix.epv) #altitude error (meters)
                   EPT = str(gpsd.fix.ept) #time error
                   Speed = str(gpsd.fix.speed) #speed (meters/sec)
                   Climb = str(gpsd.fix.climb) #change in elev
                   Head = str(gpsd.fix.track) #heading
           else:
                   Time = str(datetime.datetime.now())
                   Lat = str(24.486289)
                   Lon = str(54.353442)
                   Alt = str(30)
                   EPS = '0'
                   EPX = '0'
                   EPV = '0'
                   EPT = '0'
                   Speed = str(0)
                   Climb = str(0)
                   Head = str(0)
           if firstCycle and started:
              Time1=time.time()
              firstCycle=False
              if plotRealTime == True:
                 tempC = []
                 Humid = []
                 GNO2 = []
                 GCO = []
                 GO3 = []
                 GSO2 = []
                 plt.ion()
                 cnt=0
                 fig, ax = plt.subplots()
                 dictio=getReadings()
                 temp = round(obj.read_tmperature(),1)
                 tempi = round(obj.read_tmperature(),1)
                 NO2i = abs(getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
                 COi = abs(getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
                 O3i = abs(getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
                 SO2i = abs(getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
                 Humidityi = round(obj.read_humidity(),1) # Humidity (% H)
                 for i in range(0,26):
                     tempC.append(tempi)
                     Humid.append(Humidityi)
                     GNO2.append(NO2i)
                     GCO.append(COi)
                     GO3.append(O3i)
                     GSO2.append(SO2i)
              #Create empty array for plotting
           
           if plotRealTime == True:
             sleep (1)
             tempC.append(temp)
             Humid.append(Humidity)

             GNO2.append(NO2)
             GCO.append(CO)
             GO3.append(O3)
             GSO2.append(SO2)

             tempC.pop(0)
             Humid.pop(0)
             GNO2.pop(0)
             GCO.pop(0)
             GO3.pop(0)
             GSO2.pop(0)
             plt.pause(.1)
             Plot()
            
           temp = round(obj.read_tmperature(),4)
           if started:
              NO2.append(abs(getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp)))
              CO.append(abs(getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp)))
              O3.append(abs(getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp)))
              SO2.append(abs(getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp)))
           if includeParticleSensor == True:
              PM = alphasense.read_histogram()
              PM1.append(PM['PM1'])
              PM2.append(PM['PM2.5'])
              PM10.append(PM['PM10'])
           else:
              PM1.append(0)
              PM2.append(0)
              PM10.append(0)
              
           Temp = temp
           Humidity = str(round(obj.read_humidity(),4)) # Humidity (% H)
           BATV = str(ina.getBusVoltage_V())
           BATI = str(ina.getCurrent_mA())
           BATP = str(ina.getPower_mW())
           SOLV = str(ina2.getBusVoltage_V())
           SOLI = str(ina2.getCurrent_mA())
           SOLP = str(ina2.getPower_mW())
           
           if started:
              print 'Time till next send is', abs(interval-int(time.time()-Time1)),'seconds.'
           if Time != '': #and int(gpsd.fix.mode) != 1 and int(gpsd.fix.mode) != 0:
                   started=True
                   gpsfixed = True
                   
           else:
               print('No GPS Fix, Sending Data Without Location')
               gpsfixed = False
           if started and Time1!=None and(time.time()-Time1>=interval):
              firstCycle=True
              no2=sum(NO2) / float(len(NO2))
              co=sum(CO) / float(len(CO))
              o3=sum(O3) / float(len(O3))
              so2=sum(SO2) / float(len(SO2))
              pm1=sum(PM1) / float(len(PM1))
              pm2=sum(PM2) / float(len(PM2))
              pm10=sum(PM10) / float(len(PM10))

              NO2 = []
              CO =[]
              O3 =[]
              SO2 =[]
              PM1=[]
              PM2=[]
              PM10=[]
         
              try:
                 if saveToCSV:
                    saveToCSV()
                 if uploadToFirebase:
                    Firebase()
                 if convertToAQI:
                    AQI()
                 print('Sent!')
                 count+=1
                 print 'Packets sent since start:', count
                 print'Operating at',Temp, 'degrees celsius.'
                 print 'Location:', 'Lat', Lat, 'Lon', Lon, 'Alt', Alt
                 print 'Battery Info:', BATV, 'V', BATI, 'A', BATP, 'W'
                 print 'Solar Panel Info:', SOLV, 'V', SOLI, 'A', SOLP, 'W'

              except IOError or ConnectionError:
                 firstCycle=False
                 print ('ERROR. Did not send.')
except (KeyboardInterrupt, SystemExit): #End program when you press ctrl+c
      print ("\nEnding Thread...")
      gpsp.running = False
      if includeParticleSensor:
        alphasense.off()
      gpsp.join() # wait for the thread to finish what it's doing
      print ("Done.")
