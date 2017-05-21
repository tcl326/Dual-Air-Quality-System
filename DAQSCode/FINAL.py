#!/usr/bin/python
import struct, array, time, io, fcntl, datetime, signal, sys, random, csv, os, threading, spidev
from gps import *
from time import *
from opc import OPCN2
import time
from Adafruit_ADS1x15 import ADS1x15
from collections import defaultdict

td = open('FINALoutput.csv', 'w')
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

#PARTICLE SENSOR
spi=spidev.SpiDev()
spi.open(0,0)
spi.mode=1
spi.max_speed_hz=500000

alphasense= OPCN2(spi)

alphasense.on()

#OTHER AIR SENSORS
#Defines the number of the AFE/ISB board
boardNo='25-000009'

#Read in temp coeff table
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

def signal_handler(signal, frame):
        print ('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
#print 'Press Ctrl+C to exit'

ADS1015 = 0x00  # 12-bit ADC
ADS1115 = 0x01 # 16-bit ADC

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

  #Opens a csv and retrieves a dictionary
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
#2:WE Zero, 5:AE Zero, 6:Sensitivtity
def getConcentration(we,ae,gasName,temp):
    sensitivty=int(gasSpecifics[gasName][6])
    if (boardNo!='ISB'):
        gasNo=gasList[gasName+'-A']
    else:
        gasNo=gasList[gasName+'-B']
    if float(temp)>50:
        temp=50
    n=float(data[gasNo-1][str(round(temp, -1))])#NOTE: Will work properly at up to 50 degrees. Needs revision to work on higher temp.
    realValue=(float(we)-float(gasSpecifics[gasName][2]))-n*(float(ae)-float(gasSpecifics[gasName][5]))
    realValue=realValue/sensitivty #Gives concentration in ppb
    return realValue

header = ['Time', 'Temp', 'CO', 'O3', 'SO2', 'NO2', 'PM1', 'PM2.5', 'PM10', 'Humidity', 'Latitude', 'Longitude', 'Altitude', 'Speed', 'Climb', 'Heading', 'EPS', 'EPX', 'EPV', 'EPT']
writetd = csv.DictWriter(td, delimiter='\t', fieldnames=header)
writetd.writeheader()

if __name__ == "__main__":
   obj = HTU21D()
   gpsp = GpsPoller() # create the thread
   try:
      gpsp.start() # start it up
      while True: #int(gpsd.fix.mode) != 1: #mode: 0 = no mode, 1 = no fix, 2 = 2D fix, 3 = 3D fix
         #It may take a second or two to get good data
         #os.system('clear')
         #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
         time.sleep(1)
##         print "Temp:", round(obj.read_tmperature(),4), "C"
##         print "Humidity:", round(obj.read_humidity(),4), "% rH"
##         print 'Latitude:' , gpsd.fix.latitude
##         print 'Longitude:' , gpsd.fix.longitude
         Time = str(gpsd.utc) #datetime.datetime.now()) #Full date and time
         getReadings()
         CO = getConcentration(d['CO'][0],d['CO'][1],'CO',temp)
         O3 = getConcentration(d['O3'][0],['O3'][1],'O3',temp)
         SO2 = getConcentration(d['SO2'][0],['SO2'][1],'SO2',temp)
         NO2 = getConcentration(d['NO2'][0],['NO2'][1],'NO2',temp)
         PM1 = alphasense.read_histogram()['PM1']
         PM2 = alphasense.read_histogram()['PM2.5']
         PM10 = alphasense.read_histogram()['PM10']
         TempData = str(round(obj.read_tmperature(),4)) #Temp (Degree Celsuis)
         HumidData = str(round(obj.read_humidity(),4)) # Humidity (% H)
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
         if int(gpsd.fix.mode) != 1 and int(gpsd.fix.mode) != 0:
            print( 'Level', gpsd.fix.mode, 'Sat Fix\nNow writing data...')
            writetd.writerow({'Time': Time, 'CO': CO, 'O3': O3, 'SO2': SO2, 'NO2': NO2, 
                              'PM1': PM1, 'PM2.5': PM2, 'PM10': PM10,
                              'Temp': temp, 'Humidity': HumidData, 'Latitude': Lat, 'Longitude': Lon,
                              'Altitude': Alt, 'Speed': Speed, 'Climb': Climb, 'Heading': Head,
                              'EPS': EPS, 'EPX': EPX, 'EPV': EPV, 'EPT': EPT})
   except (KeyboardInterrupt, SystemExit): #End program when you press ctrl+c
      print ("\nEnding Thread...")
      gpsp.running = False
      gpsp.join() # wait for the thread to finish what it's doing
      print ("Done.")
   
#Check for connectivity. If connected, up to database, else, store locally
#Check stability, boot sequence, GPS
#Plug in to waqi.info for international
td.close()
sq.close()
alphasense.off()
