import random

from paho.mqtt import client as mqtt_client
from flask import Flask, request
from collections import namedtuple
import folium
import threading
import json
import struct

GPS_LOCATION_INFO = namedtuple("GPS_LOCATION_INFO", ["latitude", "longitude", "altitude", "hdop", "valid_fix", "reserved1", "num_sats", "fixtime"])




# broker = 'broker.mqttdashboard.com'
broker = '35.158.189.129'
port = 1883
topic = "petracker/location"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        payload = [f"{b:02X} " for b in msg.payload]
        print(f"Received `{payload}` from `{msg.topic}` topic")

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()



app = Flask(__name__)
start_coords = (46.9540700, 142.7360300)

coords = [46.9540700, 142.7360300]
HTML_CONTENT = """<!DOCTYPE html>
<html>
    <head>
        <title> PeTracker</title>
        <meta http-equiv="refresh" content="5" >
    </head>
    <body>
        {}
    </body>
</html>
"""

@app.route('/')
def index():
    global coords
    folium_map = folium.Map(location=coords, zoom_start=18)
    folium.Marker(
        location=coords,
        popup="Your fluffy!",
        icon=folium.Icon(icon="cloud"),
    ).add_to(folium_map)
    coords[0] += 0.0001
    coords[1] += 0.0001
    s= HTML_CONTENT.format(folium_map._repr_html_())
    return s

@app.route('/update_location', methods=['POST'])
def update_location():
    print(request.form)



if __name__ == '__main__':
    t = threading.Thread(target=run)
    t.start()
    app.run(debug=True)
