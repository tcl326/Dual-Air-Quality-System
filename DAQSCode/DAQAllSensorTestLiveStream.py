import struct, array, time, io, fcntl, datetime, signal, sys, random, csv, os, threading, spidev, datetime
from gps import *
from time import *
from time import sleep
from collections import defaultdict

#Library for Analogue to Digital Converter
from Adafruit_ADS1x15 import ADS1x15

#Library for Particle Sensor
from opc import OPCN2

#Library for Current Sensor
from Subfact_ina219 import INA219

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
    realValue=(float(we)-float(gasSpecifics[gasName][2]))-n*(float(ae)-float(gasSpecifics[gasName][5]))
    realValue=realValue/sensitivty #Gives concentration in ppb
    return realValue
gasList = {'O3-A':1, 'O3-B':2, 'SO2-A':3, 'SO2-B':4, 'NO2-A':5, 'NO2-B':6, 'NO-A':7, 'NO-B':8, 'CO-A':9, 'CO-B':10, 'H2S-A':11, 'H2S-B':12}

#Initialising Voltage and Current Sensor with their respective I2C address
ina = INA219(address=0x41)
ina1 = INA219(address=0x44)

#Initialising SPI connection for the OPC
spi=spidev.SpiDev()
spi.open(0,0)
spi.mode=1
spi.max_speed_hz=500000
sleepTime=30

#Initialising OPC object
alphasense= OPCN2(spi)

#Initialising Humidity and Temperature Sensor
humidityTemperatureSensor = HTU21D()

#Initialising ADC
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

alphasense.on()

sleep(10)
print(alphasense.read_info_string())
PM10 = alphasense.read_histogram()['PM10']
print PM10

def signal_handler(signal, frame):
        print ('Program Terminated')
        gpsp.running = False
        alphasense.off()
        gpsp.join()

        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
#print 'Press Ctrl+C to exit'

#Time = str(gpsd.utc)
def getReadings():
                numberOfSensors=4

                volts=[]
                for i in range(2*numberOfSensors):
                        if i<=3:
                                volts.append(float(adc.readADCSingleEnded(i, 1, sps)) * 6144)
                                #print (volts[i])
                        else:
                                volts.append(float(adc2.readADCSingleEnded(i-4, 1, sps)) * 6144)
                                #print (volts[i])

                d={'SO2':[0,0],'CO':[0,0],'O3':[0,0],'NO2':[0,0]}
                # for j in range(2*numberOfSensors):
                #         for i in range(len(gains)):
                #                 if (abs(float(volts[j]))< float(gains[i])/1000):
                #                         if j<=3:
                #                                 temp= adc.readADCSingleEnded(j, gains[i], sps) / 1000
                #                         else:
                #                                 temp= adc2.readADCSingleEnded(j-4, gains[i], sps) / 1000
                #                         continue
                #                 else:
                #                         volts[j]=temp
                #                         break

                d['O3'][0]=volts[0] #A0 Non-soldered
                d['O3'][1]=volts[1] #A1
                d['NO2'][0]=volts[2] #A2
                d['NO2'][1]=volts[3] #A3
                d['SO2'][0]=volts[4] #A0 Soldered
                d['SO2'][1]=volts[5] #A1
                d['CO'][0]=volts[6] #A2
                d['CO'][1]=volts[7] #A3
                return d
dictio=getReadings()
temp = round(humidityTemperatureSensor.read_tmperature(),1)
tempi = round(humidityTemperatureSensor.read_tmperature(),1)
NO2i = abs(getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
COi = abs(getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
O3i = abs(getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
SO2i = abs(getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
Humidityi = round(humidityTemperatureSensor.read_humidity(),1) # Humidity (% H)

st = datetime.datetime.utcnow()

header = ['Time','Temperature','NO2','CO','O3','SO2','Humidity','Shunt Voltage','Bus Voltage','Current','Power','Shunt Voltage1','Bus Voltage1','Current1','Power1','PM 10']

with open ('DAQDataCSVFile(%s).csv' % st, 'wb') as csvfile:
    dataWriter = csv.writer(csvfile, delimiter = ',')
    dataWriter.writerow(header)

def writeData():
    dataToWrite = []
    dictio = getReadings()
    Time = datetime.datetime.utcnow()
    temp = round(humidityTemperatureSensor.read_tmperature(),1)
    NO2 = (getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
    CO = (getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
    O3 = (getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
    SO2 = (getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
    Humidity = (round(humidityTemperatureSensor.read_humidity(),1)) # Humidity (% H)

    dataToWrite.extend([Time,temp,NO2, CO, O3, SO2, Humidity])
  #  print'GPS','Lat:', gpsd.fix.latitude, 'Lon:', gpsd.fix.longitude
    print'SENSORS\n', 'NO2', NO2, '\nCO', CO, '\nO3', O3, '\nSO2', SO2, '\nTemp', temp, '\nHumidity', Humidity



    result = ina.getBusVoltage_V()

    Shunt = ina.getShuntVoltage_mV();
    Bus = ina.getBusVoltage_V();
    Current = ina.getCurrent_mA();
    Power = ina.getPower_mW();


    print "Shunt   : %.6f mV" % Shunt
    print "Bus     : %.6f V" % Bus
    print "Current : %.6f mA" % Current
    print "Power : %.6f mW" % Power

    result = ina1.getBusVoltage_V()

    Shunt1 = ina1.getShuntVoltage_mV();
    Bus1 = ina1.getBusVoltage_V();
    Current1 = ina1.getCurrent_mA();
    Power1 = ina1.getPower_mW();

    print "Shunt   : %.6f mV" % Shunt1
    print "Bus     : %.6f V" % Bus1
    print "Current : %.6f mA" % Current1
    print "Power : %.6f mW" % Power1


    print(alphasense.read_info_string())
    PM10 = alphasense.read_histogram()['PM10']
    print PM10

    dataToWrite.extend([Shunt,Bus,Current,Power,Shunt1,Bus1,Current1,Power1,PM10])

    # for key, value in alphasense.read_histogram().items():
    #     print ("Key: {}\tValue: {}".format(key, value))

    with open ('DAQDataCSVFile(%s).csv' % st, 'a') as csvfile:
        dataWriter = csv.writer(csvfile, delimiter = ',')
        dataWriter.writerow(dataToWrite)

# while True:
#         dataToWrite = []
#         dictio = getReadings()
#         Time = datetime.datetime.utcnow()
#         temp = round(humidityTemperatureSensor.read_tmperature(),1)
#         NO2 = (getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
#         CO = (getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
#         O3 = (getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
#         SO2 = (getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
#         Humidity = (round(humidityTemperatureSensor.read_humidity(),1)) # Humidity (% H)
#
#         dataToWrite.extend([Time,temp,NO2, CO, O3, SO2, Humidity])
#       #  print'GPS','Lat:', gpsd.fix.latitude, 'Lon:', gpsd.fix.longitude
#         print'SENSORS\n', 'NO2', NO2, '\nCO', CO, '\nO3', O3, '\nSO2', SO2, '\nTemp', temp, '\nHumidity', Humidity
#
#
#
#         result = ina.getBusVoltage_V()
#
#         Shunt = ina.getShuntVoltage_mV();
#         Bus = ina.getBusVoltage_V();
#         Current = ina.getCurrent_mA();
#         Power = ina.getPower_mW();
#
#
#         print "Shunt   : %.6f mV" % Shunt
#         print "Bus     : %.6f V" % Bus
#         print "Current : %.6f mA" % Current
#         print "Power : %.6f mW" % Power
#
#         result = ina1.getBusVoltage_V()
#
#         Shunt1 = ina1.getShuntVoltage_mV();
#         Bus1 = ina1.getBusVoltage_V();
#         Current1 = ina1.getCurrent_mA();
#         Power1 = ina1.getPower_mW();
#
#         print "Shunt   : %.6f mV" % Shunt1
#         print "Bus     : %.6f V" % Bus1
#         print "Current : %.6f mA" % Current1
#         print "Power : %.6f mW" % Power1
#
#
#         print(alphasense.read_info_string())
#         PM10 = alphasense.read_histogram()['PM10']
#         print PM10
#
#         dataToWrite.extend([Shunt,Bus,Current,Power,Shunt1,Bus1,Current1,Power1,PM10])
#
#         # for key, value in alphasense.read_histogram().items():
#         #     print ("Key: {}\tValue: {}".format(key, value))
#
#         with open ('DAQDataCSVFile(%s).csv' % st, 'a') as csvfile:
#             dataWriter = csv.writer(csvfile, delimiter = ',')
#             dataWriter.writerow(dataToWrite)
#
#         sleep(sleepTime)
#
# alphasense.off()

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ((axTemp,axNO2,axCO),(axPM10,axO3,axSO2)) = plt.subplots(2,3, sharex=True)


# yData1 = [0]*120
# line1, = ax1.plot(yData1)
# ax1.set_ylim(0, 300)
# yMax1 = [0]
#
# # fig2, ax2 = plt.subplots(2,2)
# yData2 = [0]*120
# line2, = ax2.plot(yData1)
# ax2.set_ylim(0, 300)
# yData2 = [0]*120
# yMax2 = [0]

tempData = [0]*120
NO2Data = [0]*120
COData = [0]*120
O3Data = [0]*120
SO2Data = [0]*120
PM10Data = [0]*120

xValue = np.linspace(120,0,120)

axO3.set_xlabel('Time Passed (s)')
axSO2.set_xlabel('Time Passed (s)')
axPM10.set_xlabel('Time Passed (s)')

axTemp.set_ylabel('Temperature (C)')
axNO2.set_ylabel('Nitrogen Dioxide (ppb)')
axCO.set_ylabel('Carbon Monoxide (ppb)')
axO3.set_ylabel('Ozone (ppb)')
axSO2.set_ylabel('Sulfur Dioxide (ppb)')
axPM10.set_ylabel('PM10 (ug/m3)')

lineTemp, = axTemp.plot(xValue,tempData)
lineNO2, = axNO2.plot(xValue,NO2Data)
lineCO, = axCO.plot(xValue,COData)
lineO3, = axO3.plot(xValue,O3Data)
lineSO2, = axSO2.plot(xValue,SO2Data)
linePM10, = axPM10.plot(xValue,PM10Data)



yMax = [0]*6
def init():
    lineTemp.set_ydata(np.ma.array(tempData, mask=True))
    lineNO2.set_ydata(np.ma.array(NO2Data, mask=True))
    lineCO.set_ydata(np.ma.array(COData, mask=True))
    lineO3.set_ydata(np.ma.array(O3Data, mask=True))
    lineSO2.set_ydata(np.ma.array(SO2Data, mask=True))
    linePM10.set_ydata(np.ma.array(PM10Data, mask=True))
    return lineTemp,lineNO2,lineCO,lineO3,lineSO2,linePM10,
def update(data):
    redraw = False
    tempG, NO2G, COG, O3G, SO2G, PM10G = data
    tempData.append(tempG)
    NO2Data.append(NO2G)
    COData.append(COG)
    O3Data.append(O3G)
    SO2Data.append(SO2G)
    PM10Data.append(PM10G)
    tempData.pop(0)
    NO2Data.pop(0)
    COData.pop(0)
    O3Data.pop(0)
    SO2Data.pop(0)
    PM10Data.pop(0)

    lineTemp.set_ydata(tempData)
    lineNO2.set_ydata(NO2Data)
    lineCO.set_ydata(COData)
    lineO3.set_ydata(O3Data)
    lineSO2.set_ydata(SO2Data)
    linePM10.set_ydata(PM10Data)
    # yData1.append(data)
    # yData1.pop(0)
    # line1.set_ydata(yData1)
    if tempG > yMax[0]:
        yMax[0] = tempG*1.5
        axTemp.set_ylim(0,yMax[0])
        redraw = True
    if NO2G > yMax[1]:
        yMax[1] = NO2G*1.5
        axNO2.set_ylim(0,yMax[1])
        redraw = True
    if COG > yMax[2]:
        yMax[2] = COG*1.5
        axCO.set_ylim(0,yMax[2])
        redraw = True
    if O3G > yMax[3]:
        yMax[3] = O3G*1.5
        axO3.set_ylim(0,yMax[3])
        redraw = True
    if SO2G > yMax[4]:
        yMax[4] = SO2G*1.5
        axSO2.set_ylim(0,yMax[4])
        redraw = True
    if PM10G > yMax[5]:
        yMax[5] = PM10G*1.5
        axPM10.set_ylim(0,yMax[5])
        redraw = True
    if redraw:
        plt.draw()
    return lineTemp,lineNO2,lineCO,lineO3,lineSO2,linePM10,

def data_gen():
    while True:
        dataToGraph = []
        dictio = getReadings()
        temp = round(humidityTemperatureSensor.read_tmperature(),1)
        NO2 = abs(getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp))
        CO = abs(getConcentration(dictio['CO'][0], dictio['CO'][1],'CO',temp))
        O3 = abs(getConcentration(dictio['O3'][0], dictio['O3'][1],'O3',temp))
        SO2 = abs(getConcentration(dictio['SO2'][0], dictio['SO2'][1],'SO2',temp))
        print(alphasense.read_info_string())
        PM10 = alphasense.read_histogram()['PM10']
        print PM10
        print NO2
        print CO
        print O3
        print SO2
        dataToGraph.extend([temp, NO2, CO, O3, SO2, PM10])
        yield dataToGraph

ani = animation.FuncAnimation(fig, update, frames = data_gen, init_func = init, blit=True,interval=1000)
mng = plt.get_current_fig_manager()
### works on Ubuntu??? >> did NOT working on windows
mng.resize(*mng.window.maxsize())
# mng.window.state('zoomed') #works fine on Windows!
plt.show()

# def update(data):
#     yData.append(data)
#     line.set_ydata(yData)
#     return line,
#
# def dataGen():
#     while True:
#         yield (getConcentration(dictio['NO2'][0], dictio['NO2'][1], 'NO2',temp)
#
#
# ani = animation.FuncAnimation(fig, update, dataGen, interval=30000)
# plt.show()
