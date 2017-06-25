from OPCN2 import OPCN2
import time

alphasenseOPC = OPCN2()
alphasenseOPC.begin()
while True:
    try:
        print alphasenseOPC.getPM10()
        time.sleep(1)
    except KeyboardInterrupt:
    	print(alphasenseOPC.stop())
        quit()
    except StopIteration:
		session = None
		print "OPCN2 has terminated"
