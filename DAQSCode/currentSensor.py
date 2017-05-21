from Subfact_ina219 import INA219

ina = INA219(address=0x41)
result = ina.getBusVoltage_V()

print "Shunt   : %.6f mV" % ina.getShuntVoltage_mV()
print "Bus     : %.6f V" % ina.getBusVoltage_V()
print "Current : %.6f mA" % ina.getCurrent_mA()
print "Power : %.6f mW" % ina.getPower_mW()



ina = INA219(address=0x44)
result = ina.getBusVoltage_V()

print "Shunt   : %.6f mV" % ina.getShuntVoltage_mV()
print "Bus     : %.6f V" % ina.getBusVoltage_V()
print "Current : %.6f mA" % ina.getCurrent_mA()
print "Power : %.6f mW" % ina.getPower_mW()
