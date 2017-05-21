import aqi

pm25aqi = aqi.to_aqi([(aqi.POLLUTANT_PM25, '12')])
pm10aqi = aqi.to_aqi([(aqi.POLLUTANT_PM10, '12')])
pm1aqi = aqi.to_aqi([(aqi.POLLUTANT_PM10, '12')])
pm25aqi = aqi.to_aqi([(aqi.POLLUTANT_PM25, '12')])
##   (aqi.POLLUTANT_PM10, '25')

print(pm25aqi)
print(pm10aqi)
print(pm10aqi)
