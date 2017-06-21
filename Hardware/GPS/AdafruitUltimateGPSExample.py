from AdafruitUltimateGPS import GPSD
import time

adafruitGPS = GPSD()
adafruitGPS.begin()
while True:
    try:
        print adafruitGPS.getTime()
        time.sleep(1)
    except KeyboardInterrupt:
    	print(adafruitGPS.stop())
        quit()
    except StopIteration:
		session = None
		print "GPSD has terminated"
