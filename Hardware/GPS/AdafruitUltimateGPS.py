import gps
import os
import threading
import time
import math

class GPSD(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        os.system("sudo gpsd /dev/ttyS0 -F /var/run/gpsd.sock")
        self.gpsd = gps.gps("localhost", "2947")
        self.mode = 1
        self.time = float('nan')
        self.ept = float('nan')
        self.latitude = self.longitude = 0.0
        self.epx = float('nan')
        self.epy = float('nan')
        self.altitude = float('nan')         # Meters
        self.epv = float('nan')
        self.track = float('nan')            # Degrees from true north
        self.speed = float('nan')            # Knots
        self.climb = float('nan')            # Meters per second
        self.epd = float('nan')
        self.eps = float('nan')
        self.epc = float('nan')

        self.heading = float('nan')
        self.longitudeErr = float('nan')
        self.latitudeErr = float('nan')
        self.altitudeErr = float('nan')
        self.courseErr = float('nan')
        self.speedErr = float('nan')
        self.satellite = 0
        self.report = {}

    def run(self):
        self._is_running = True
        self.gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        while self._is_running:
            self.report = self.gpsd.next()
            if (self.isnan(self.gpsd.fix.time) == False):
                self.time = self.gpsd.fix.time
            if (self.gpsd.fix.mode >= 2 and self.isnan(self.gpsd.fix.latitude) == False):
                self.latitude = self.gpsd.fix.latitude
            if (self.gpsd.fix.mode >= 2 and self.isnan(self.gpsd.fix.longitude) == False):
                self.longitude = self.gpsd.fix.longitude
            if (self.gpsd.fix.mode >= 3 and self.isnan(self.gpsd.fix.altitude) == False):
                self.altitude = self.gpsd.fix.altitude
            if (self.gpsd.fix.mode >= 2 and self.isnan(self.gpsd.fix.track) == False):
                self.speed = self.gpsd.fix.speed
            #give only the true heading
            if (self.gpsd.fix.mode >= 2 and self.isnan(self.gpsd.fix.track) == False):
                self.heading = self.gpsd.fix.track
            if (self.gpsd.fix.mode >= 3 and self.isnan(self.gpsd.fix.climb) == False):
                self.climb = self.gpsd.fix.climb
            if (self.isnan(self.gpsd.fix.epx) == False):
                self.longitudeErr = self.gpsd.fix.epx
            if (self.isnan(self.gpsd.fix.epy) == False):
                self.latitudeErr = self.gpsd.fix.epy
            if (self.isnan(self.gpsd.fix.epv) == False):
                self.altitudeErr = self.gpsd.fix.epv
            if (self.isnan(self.gpsd.fix.epd) == False):
                self.courseErr = self.gpsd.fix.epd
            if (self.isnan(self.gpsd.fix.eps) == False):
                self.speedErr = self.gpsd.fix.eps
            self.satellite = self.gpsd.status
            time.sleep(1)
            # print "1"

        # print "stopping"
        return
            # print report
    def isnan(self, x):
        return str(x) == 'nan'
    def begin(self):
        self.start()
    def stop(self):
        self._is_running = False
        # self.terminate()
        self.join(timeout = 1)
        self.close()
        return self._is_running
    def getReport(self):
        return self.report
    def getLocation(self):
        return  self.latitude, self.longitude
    def getLatitude(self):
        return self.latitude
    def getLongitude(self):
        return self.longitude
    def getSatellite(self):
        return self.satellite
    def getSpeed(self):
        return self.speed
    def getHeading(self):
        return self.heading
    def getClimb(self):
        return self.climb
    def getLongitudeErr(self):
        return self.longitudeErr
    def getLatitudeErr(self):
        return self.latitudeErr
    def getCourseErr(self):
        return self.courseErr
    def getSpeedErr(self):
        return self.speedErr
    def getTime(self):
        return self.time
    def close(self):
        os.system("sudo killall gpsd")
