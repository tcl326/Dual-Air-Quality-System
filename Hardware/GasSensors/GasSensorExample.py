from GasSensors import GasSensors
from SHT31D import SHT31D
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
		print "alphasenseGasSensors has terminated"
