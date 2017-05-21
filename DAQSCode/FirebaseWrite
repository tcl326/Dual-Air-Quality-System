import json
import urllib2
import time
import calendar


url = 'https://air-quality-readings.firebaseio.com/readings.json'

postdata = {
   'date': str(calendar.timegm(time.gmtime())),
   'AQI': str(12)

   }

req = urllib2.Request(url)
req.add_header('Content-Type', 'application/json')
data = json.dumps(postdata)


while True:
   response = urllib2.urlopen(req,data)
   time.sleep(5)
