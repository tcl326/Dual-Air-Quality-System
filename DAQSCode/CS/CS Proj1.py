

#PLOTLY
import plotly.plotly as py # plotly library
from plotly.graph_objs import Scatter, Layout, Figure # plotly graph objects
import time # timer functions
#import readadc # helper functions to read ADC from the Raspberry Pi
py.sign_in('will0990', 'tnhp51vobj')

trace1 = Scatter(
    x=[],
    y=[],
    stream=dict(
        token='5noyyuzs0n',
        maxpoints=20
    )
)

layout = Layout(
    title='DAQSRTP'
)

fig = Figure(data=[trace1], layout=layout)

print py.plot(fig, filename='fig')

##sensor_pin = 0
##readadc.initialize()

stream = py.Stream('5noyyuzs0n')
stream.open()

while True:
    sensor_data = 5 #readadc.readadc(sensor_pin, readadc.PINS.SPICLK, readadc.PINS.SPIMOSI, readadc.PINS.SPIMISO, readadc.PINS.SPICS)
    stream.write({'x': datetime.datetime.now(), 'y': sensor_data})
    time.sleep(1) # delay between stream posts
