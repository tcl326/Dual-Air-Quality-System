import SHT31D

sensor = SHT31D.SHT31D(0x45)
print(sensor.heaterEnable())
print(bin(sensor.readStatus()))
print(sensor.heaterDisable())
print(bin(sensor.readStatus()))
# print("(Temperature,Humidity):", sensor.readTemperatureAndHumidity());
# print("Temperature:", sensor.readTemperature());
# print("Humidity:", sensor.readHumidity());

# print hex(sensor.CRC8(bin(0xBE ^ 0xFF)[2:]))
# print hex(sensor.CRC8(bin(0xEF)[2:]))
