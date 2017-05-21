import spidev
from opc import OPCN2
from time import sleep

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode = 1
spi.max_speed_hz = 500000

alphasense = OPCN2(spi)

# Turn the opc ON
alphasense.on()

# Read the information string
print (alphasense.read_info_string())

# Read the histogram
print (alphasense.read_histogram())

# Turn the opc OFF
alphasense.off()
