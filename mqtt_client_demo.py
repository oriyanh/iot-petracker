import random
from dataclasses import dataclass

from paho.mqtt import client as mqtt_client
from flask import Flask, request, jsonify
from collections import namedtuple
import folium
import threading
import json
import struct
import requests
GPS_LOCATION_INFO = namedtuple("GPS_LOCATION_INFO", ["latitude", "longitude", "altitude", "hdop", "valid_fix_reserved1_num_sats", "fixtime"])

@dataclass
class GPSLocation:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    hdop: float = 0.0
    valid_fix: int = 0
    num_sats: int = 0
    fix_time: str = ""


current_location = GPSLocation()

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
        if len(msg.payload) < 23:
            print("MQTT short message, doing nothing")
            return
        loc = GPS_LOCATION_INFO(*struct.unpack("<3iBB9s", msg.payload[:23]))
        global current_location
        current_location.latitude = loc.latitude * 1e-8
        current_location.longitude = loc.longitude * 1e-8
        current_location.altitude = loc.altitude * 1e-2
        current_location.hdop = loc.hdop * 0.2
        current_location.fix_time = loc.fixtime.decode()
        current_location.num_sats = loc.valid_fix_reserved1_num_sats & 0b00011111
        current_location.valid_fix = (loc.valid_fix_reserved1_num_sats & 0b11000000) >> 6

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
    global coords, current_location
    coords[0] = current_location.latitude
    coords[1] = current_location.longitude
    folium_map = folium.Map(location=coords, zoom_start=18)
    folium.Marker(
        location=coords,
        popup="Your fluffy!",
        icon=folium.Icon(icon="cloud"),
    ).add_to(folium_map)
    s= HTML_CONTENT.format(folium_map._repr_html_())
    return s

if __name__ == '__main__':
    t = threading.Thread(target=run)
    t.start()
    app.run(debug=True)
