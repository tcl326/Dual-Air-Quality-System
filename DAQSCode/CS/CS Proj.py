

#PLOTLY
import plotly.plotly as py # plotly library
from plotly.graph_objs import Scatter, Layout, Figure # plotly graph objects
import time # timer functions
import readadc # helper functions to read ADC from the Raspberry Pi
py.sign_in(will0990, e84kzbv6bs)

trace1 = Scatter(
    x=[],
    y=[],
    stream=dict(
        token=stream_token,
        maxpoints=200
    )
)

layout = Layout(
    title='Raspberry Pi Streaming Sensor Data'
)

fig = Figure(data=[trace1], layout=layout)

print py.plot(fig, filename='DAQSFULLSENSORSTOCSV.py')

sensor_pin = 0
readadc.initialize()

stream = py.Stream(stream_token)
stream.open()

while True:
    sensor_data = readadc.readadc(sensor_pin, readadc.PINS.SPICLK, readadc.PINS.SPIMOSI, readadc.PINS.SPIMISO, readadc.PINS.SPICS)
    stream.write({'x': datetime.datetime.now(), 'y': temp})
    time.sleep(1) # delay between stream posts
