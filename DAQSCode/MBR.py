import struct, array, time, io, fcntl, datetime, signal, sys, random, csv, os, threading, spidev
from gps import *
from time import *
from opc import OPCN2
import spidev
from time import sleep
from Adafruit_ADS1x15 import ADS1x15
from collections import defaultdict
import os
import matplotlib.pyplot as plt

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
        print ('Program Terminated')
        gpsp.running = False
        alphasense.off()
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

#MATPLOTLIB
tempC = []
Humid = []
GPM = []
GNO2 = []
GCO = []
GO3 = []
GSO2 = []
plt.ion()
cnt=0
#y1 = (0,10)
#y2 = (0,1)
fig, ax = plt.subplots()
##ax2 = ax.twinx()
##ax.set_ylim(0,20)
##ax2.set_ylim(0,3)
def plotNow():
    plt.clf()
    #ylim(20,25)
    plt.ax = plt.subplot(411)
    plt.ax2 = plt.subplot(412, sharex = ax)
    plt.ax3 = plt.subplot(413, sharex = ax)
    plt.ax4 = plt.subplot(414, sharex = ax)

##    plt.title('Temp')
##    plt.ax2.title('Humidity')
##    plt.ax3.title('Particulate Matter')
    plt.grid(True)
    plt.ylabel('Data Readings')
    plt.ax.plot(tempC, 'rx-', label='Temp (Deg C)')
    plt.ax2.plot(Humid, 'cx-', label='Humidity (%)')
    plt.ax3.plot(GPM, 'bx-', label='PM (PPB)')
    plt.ax4.plot(GNO2, 'bx-', label='NO2 (AQI)')
    plt.ax4.plot(GCO, 'gx-', label='CO (AQI)')
    plt.ax4.plot(GO3, 'mx-', label='O3 (AQI)')
    plt.ax4.plot(GSO2, 'kx-', label='SO2 (AQI)')
    plt.ax.legend(loc='upper left')
    plt.ax2.legend(loc='upper left')
    plt.ax3.legend(loc='upper left')
    plt.ax4.legend(loc='upper left')
    #plt.plot(tempC, Humid, GNO2, GCO, GO3, GSO2)
    plt.show()



#INSTANTIATING EVERYTHING
obj = HTU21D()
spi= spidev.SpiDev()
spi.open(0,0)
spi.mode=1
spi.max_speed_hz=500000
alphasense = OPCN2(spi)
alphasense.on()


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
temp = round(obj.read_tmperature(),1)
tempi = round(obj.read_tmperature(),1)
NO2i = abs(getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
COi = abs(getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
O3i = abs(getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
SO2i = abs(getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
Humidityi = round(obj.read_humidity(),1) # Humidity (% H)
PMi = alphasense.read_histogram()['PM10']

for i in range(0,26):
    tempC.append(tempi)
    Humid.append(Humidityi)
    GPM.append(PMi)
    GNO2.append(NO2i)
    GCO.append(COi)
    GO3.append(O3i)
    GSO2.append(SO2i)

while True:
        temp = round(obj.read_tmperature(),1)
        NO2 = abs(getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
        CO = abs(getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
        O3 = abs(getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
        SO2 = abs(getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
        PM = alphasense.read_histogram()['PM10']
        Humidity = (round(obj.read_humidity(),1)) # Humidity (% H)
       
        print'SENSORS\n', 'NO2', NO2, '\nCO', CO, '\nO3', O3, '\nSO2', SO2, '\nTemp', temp, '\nHumidity', Humidity
        tempC.append(temp)#-tempi)
        Humid.append(Humidity)#-Humidityi)
        GPM.append(PM)
        GNO2.append(NO2)
        GCO.append(CO)
        GO3.append(O3)
        GSO2.append(SO2)
        GPM.pop(0)
        tempC.pop(0)
        Humid.pop(0)
        GNO2.pop(0)
        GCO.pop(0)
        GO3.pop(0)
        GSO2.pop(0)
        plotNow()
        plt.pause(.1)


alphasense.off()
