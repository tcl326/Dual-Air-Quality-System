import time
import sys
import json
from azure.servicebus import ServiceBusService
key_name = "D1"
key_value = "9aZsaZudW0xgkDKC8XXbC7pYAlohLGrUb4ZZLj/Xl8o="
sbs = ServiceBusService("dualdots-ns",shared_access_key_name=key_name, shared_access_key_value=key_value)
value=1

event={'Location':1,'DeviceId':2,'Temperature':3}
while(value<11):
    value+=1
    print('sending...')
    #sbs.send_event('ehdevices', '{ "DeviceId": "smokerpi", "Temperature": "37"guid":"62X74059-A444-4797-8A7E-526C3EF9D64B","organization":"My Org Name","displayname":"Sensor Name","location":"Sensor Location","measurename":"Temperature","unitofmeasure":"F","value":74.0 }')
    #sbs.send_event('ehdevices', 'Temperature: '+str(value//10))
    sbs.send_event('ehdevices',json.dumps(event))
    print (json.dumps(event))
    print('sent!')

