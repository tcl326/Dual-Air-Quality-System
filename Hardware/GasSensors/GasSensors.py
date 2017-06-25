from Adafruit_ADS1x15 import ADS1115
import threading
import time
import csv
from collections import defaultdict

class GasSensors(threading.Thread):
    def __init__(self, O3BoardNo = 'ISB', SO2BoardNo = 'ISB', NO2BoardNo = 'ISB', COBoardNo = 'ISB'):
        threading.Thread.__init__(self)
        self.O3boardNo = O3BoardNo; #this needs to change to reflect proper callibration value
        self.SO2BoardNo = SO2BoardNo;
        self.NO2BoardNo = NO2BoardNo;
        self.COBoardNo = COBoardNo;
        self.ADS1115 = 0x01
        self.adc = ADS1115(address=0x48)
        self.adc2 = ADS1115(address=0x49)
        self.sps = 8
        self.tempAndHumCalibrationData = []
        self.gasOffsetsAndSensitivity = []
        self.NO2 = 0
        self.O3 = 0
        self.CO = 0
        self.SO2 = 0
        self.temperature = 0
        self.gasADCReadings = {'SO2':[0,0],'CO':[0,0],'O3':[0,0],'NO2':[0,0]}

    def begin(self):
        self.readTDDCFile()
        self.getOffsetsAndSensitivity()
        self.start();
    def updateTemperature(self, temperature):
        self.temperature = temperature
    def run(self):
        self._is_running = True;
        while self._is_running:
            self.getReadings()
            self.updateCalibratedValue()
            time.sleep(2)
    def stop(self):
        self._is_running = False
        # self.terminate()
        # self.close()
        self.join(timeout = 1)
        return self._is_running
    def readTDDCFile(self):
        #WY Read in header of CSV file.
        sq = open('tddc.csv', mode='r')
        hdr = sq.readline()
        hdr = hdr.strip().split(',')
        #WY Create list (data) with each positions full data (temp and coefficient) as a single entry.
        # self.tempAndHumCalibrationData = []
        #WY Store each positions full data as a dictionary in each list position (keys = temp, values = coeff)
        for line in sq:
            row = line.strip().split(',')
            d = {}
            for i in range(len(hdr)):
                key = hdr[i]
                value = row[i]
                d[key] = value
            self.tempAndHumCalibrationData.append(d)
        sq.close()
    def getOffsetsAndSensitivity(self):
        # i=1
        fileName = self.O3boardNo+'.csv'
        csv=open(fileName, 'r')
        hdr = csv.readline()
        hdr = hdr.strip().split(',')
        self.gasOffsetsAndSensitivity = defaultdict(list)
        lines = csv.readlines()
        for line in lines:
            row = line.strip().split(',')
            for i in range(1,len(hdr)):
                key = str(hdr[i])
                value = str(row[i])
                self.gasOffsetsAndSensitivity[key].append(value)

        csv.close()
        return self.gasOffsetsAndSensitivity
    def getConcentration(self,we,ae,gasName):
        gasSpecifics=self.getOffsetsAndSensitivity()
        gasList = {'O3-A':1, 'O3-B':2, 'SO2-A':3, 'SO2-B':4, 'NO2-A':5, 'NO2-B':6, 'NO-A':7, 'NO-B':8, 'CO-A':9, 'CO-B':10, 'H2S-A':11, 'H2S-B':12}
        sensitivty=float(gasSpecifics[gasName][6])
        if ('ISB' not in self.O3boardNo):
            gasNo=gasList[gasName+'-A']
        else:
            gasNo=gasList[gasName+'-B']
        if float(self.temperature )>50:
            self.temperature =50
        n=float(self.tempAndHumCalibrationData[gasNo-1][str(int(round(self.temperature , -1)))])#NOTE: Will work properly at up to 50 degrees. Needs revision to work on higher temp.
        realValue=(float(we)-float(gasSpecifics[gasName][2]))-n*(float(ae)-float(gasSpecifics[gasName][5]))
        realValue=realValue/sensitivty #Gives concentration in ppb
        return realValue
    def getReadings(self):
        numberOfSensors=4
        volts=[]
        for i in range(2*numberOfSensors):
            if i<=3:
                volts.append(float(self.adc.read_adc(i, 1, self.sps)) * 6144)
                    #print (volts[i])
            else:
                volts.append(float(self.adc2.read_adc(i-4, 1, self.sps)) * 6144)
                        #print (volts[i])
        self.gasADCReadings['O3'][0]=volts[0] #A0 Non-soldered
        self.gasADCReadings['O3'][1]=volts[1] #A1
        self.gasADCReadings['NO2'][0]=volts[2] #A2
        self.gasADCReadings['NO2'][1]=volts[3] #A3
        self.gasADCReadings['SO2'][0]=volts[4] #A0 Soldered
        self.gasADCReadings['SO2'][1]=volts[5] #A1
        self.gasADCReadings['CO'][0]=volts[6] #A2
        self.gasADCReadings['CO'][1]=volts[7] #A3

    def updateCalibratedValue(self):
        self.NO2 = abs(self.getConcentration(self.gasADCReadings['NO2'][0], self.gasADCReadings['NO2'][1], 'NO2' ))
        self.CO = abs(self.getConcentration(self.gasADCReadings['CO'][0], self.gasADCReadings['CO'][1],'CO' ))
        self.O3 = abs(self.getConcentration(self.gasADCReadings['O3'][0], self.gasADCReadings['O3'][1],'O3'))
        self.SO2 = abs(self.getConcentration(self.gasADCReadings['SO2'][0], self.gasADCReadings['SO2'][1],'SO2'))

    def getNO2(self):
        return self.NO2
    def getCO(self):
        return self.CO
    def getO3(self):
        return self.O3
    def getSO2(self):
        return self.SO2
