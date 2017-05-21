import time
import sys
import json
import csv 
from azure.servicebus import ServiceBusService
key_name = "D1"
key_value = "9aZsaZudW0xgkDKC8XXbC7pYAlohLGrUb4ZZLj/Xl8o="
sbs = ServiceBusService("dualdots-ns",shared_access_key_name=key_name, shared_access_key_value=key_value)
csvfile=open('AirMockData111215.csv','r')
fieldnames=("DeviceId","Time","O3","SO2","NO2","CO","PM10","PM1","PM2.5","LATITUDE","LONGITUDE")

#sbs.send_event('ehdevices',event)

reader=csv.DictReader(csvfile,fieldnames)
i=0
for row in reader:
    if (i!=0):
        print (i)
        print('sending...')
        print(json.dumps(row))
        sbs.send_event('ehdevices',(json.dumps(row)))
        print('sent!')
    i+=1

