import spidev
import opc
from time import sleep
import threading


class OPCN2(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # Open a SPI connection on CE0
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        # Set the SPI mode and clock speed
        self.spi.mode = 1
        self.spi.max_speed_hz = 500000
        self.SFR = 0
        self.PM25 = 0
        self.PM1 = 0
        self.PM10 = 0
        self.samplingPeriod = 0
        try:
            self.alpha = opc.OPCN2(self.spi)
        except Exception as e:
            print ("Startup Error: {}".format(e))
    def begin(self):
        self.alpha.on()
        sleep(2)
        for i in range(5):
            if "OPC-N2 FirmwareVer" not in self.alpha.read_info_string():
                if i == 4:
                    raise NameError('Sensor not found')
                sleep(2)
                self.reset()
            else:
                self.start()
                break;
    def run(self):
        self._is_running = True
        while self._is_running:
            dataDict = self.alpha.histogram()
            if 'SFR' in dataDict:
                # print ("SFR Is in")
                self.SFR = dataDict['SFR']
            if 'PM2.5' in dataDict:
                self.PM25 = dataDict['PM2.5']
            if 'PM1' in dataDict:
                self.PM1 = dataDict['PM1']
            if 'PM10' in dataDict:
                self.PM10 = dataDict['PM10']
            if 'Sampling Period' in dataDict:
                self.samplingPeriod = dataDict['Sampling Period']
            # for key, value in dataDict:
            #     print (key, value)
            sleep(2)
    def reset(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        # Set the SPI mode and clock speed
        self.spi.mode = 1
        self.spi.max_speed_hz = 500000
        try:
            self.alpha = opc.OPCN2(self.spi)
        except Exception as e:
            print ("Startup Error: {}".format(e))
        self.alpha.on()
        sleep(2)
    def getSFR(self):
        return self.SFR
    def getPM25(self):
        return self.PM25
    def getPM1(self):
        return self.PM1
    def getPM10(self):
        return self.PM10
    def getSamplingPeriod(self):
        return self.samplingPeriod
    def close(self):
        self.alpha.off()
    def stop(self):
        self._is_running = False
        # self.terminate()
        self.close()
        self.join(timeout = 1)
        return self._is_running

#
# alphasenseOPC = OPCN2()
# alphasenseOPC.begin()
#
# # Turn on the OPC
# alpha.on()
# sleep(2)
# print(alpha.read_info_string())
#
# # Read the histogram and print to console
# for i in range (10):
#     for key, value in alpha.histogram().items():
#         print ("Key: {}\tValue: {}".format(key, value))
#     sleep(2)
#
# # Shut down the opc
# alpha.off()
