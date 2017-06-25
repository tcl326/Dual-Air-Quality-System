from GasSensors import GasSensors
import time

alphasenseGasSensors = GasSensors()
alphasenseGasSensors.begin()
while True:
    try:
        print alphasenseGasSensors.getSO2()
        time.sleep(1)
    except KeyboardInterrupt:
    	print(alphasenseGasSensors.stop())
        quit()
    except StopIteration:
		session = None
		print "OPCN2 has terminated"
