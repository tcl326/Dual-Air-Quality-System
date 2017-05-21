import spidev
from opc import OPCN2
from time import sleep

spi=spidev.SpiDev()
spi.open(0,0)
spi.mode=1
spi.max_speed_hz=500000
sleepTime=2

alphasense= OPCN2(spi)

alphasense.on()
sleep(sleepTime)

print(alphasense.read_info_string())
sleep(sleepTime)

for i in range (10):
    print(alphasense.read_histogram()['PM10'])
    print(alphasense.read_histogram())
    sleep(sleepTime)

alphasense.off()
