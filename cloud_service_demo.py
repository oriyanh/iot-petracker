import random
import sys
import time
import traceback
from dataclasses import dataclass, asdict
import datetime as dt
from paho.mqtt import client as mqtt_client
from flask import Flask
from collections import namedtuple
import folium
import threading
import json
import struct

###  MQTT Client ###

GPS_LOCATION_INFO = namedtuple("GPS_LOCATION_INFO", ["latitude", "longitude", "altitude", "hdop", "valid_fix_reserved1_num_sats", "fix_time"])

@dataclass
class GPSLocation:
    latitude: str = "0.0"
    longitude: str = "0.0"
    altitude: float = 0.0
    hdop: float  = 0.0
    valid_fix: int = 0
    num_sats: int = 0
    fix_time: str = ""

current_location = GPSLocation()

# broker = 'broker.mqttdashboard.com'
# broker = '35.158.189.129'
broker = '52.29.249.84'
port = 1883
location_raw_topic = "petracker/location_raw"
location_parsed_topic = "petracker/location"
distress_topic = "petracker/distress"
distress = False

# generate client ID with pub prefix randomly
client_id = f'iot-petracker-mqtt-{random.randint(0, 100)}'
mqtt_is_connected = False
def connect_mqtt() -> mqtt_client.Client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            global mqtt_is_connected
            mqtt_is_connected = True
        else:
            print("Failed to connect, return code %d", rc)

    client = mqtt_client.Client(client_id, clean_session=True)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        try:
            print(f"[{msg.topic} recv] RAW payload ({len(msg.payload)}b):\t`{' '.join(f'{b:02X}' for b in msg.payload)}`")
            if msg.topic != location_raw_topic:
                payload = msg.payload.decode()
                print(f"[{msg.topic}] `{payload}`")
                if msg.topic == distress_topic:
                    global distress
                    distress = payload == "enable"

                return

            if len(msg.payload) < 23:
                print("\tMQTT short message, doing nothing")
                return

            loc = GPS_LOCATION_INFO(*struct.unpack("<iiiBB9s", msg.payload[:23]))  # Ignoring 24th byte which is NULL character
            global current_location
            lat_str = str(loc.latitude)
            current_location.latitude = f"{lat_str[:-7]}.{lat_str[-7:]}"
            lon_str = str(loc.longitude)
            current_location.longitude = f"{lon_str[:-7]}.{lon_str[-7:]}"
            alt_str = str(loc.altitude)
            current_location.altitude = f"{alt_str[:-2]}.{alt_str[-2:]}"
            hdop_str = str(loc.hdop*2).rjust(2, '0')
            current_location.hdop = f"{hdop_str[:-1]}.{hdop_str[-1]}"
            fix_time_str = loc.fix_time.decode()
            utc_parse = dt.datetime.strptime(fix_time_str.rjust(9, '0'), "%H%M%S%f")
            current_location.fix_time = f"{utc_parse.hour:02}:{utc_parse.minute:02}:{utc_parse.second:02}"
            current_location.num_sats = (loc.valid_fix_reserved1_num_sats & 0b11111000) >> 3
            current_location.valid_fix = (loc.valid_fix_reserved1_num_sats & 0b00000011)
            print(f"current_location={asdict(current_location)}")
            publish(client)
        except Exception as e:
            print(f"Exception raised during `on_message`: {e}, traceback:")
            traceback.print_exc()

    client.subscribe([(location_raw_topic, 2), (distress_topic, 2)])
    client.on_message = on_message

def publish(client):
    global current_location
    msg = json.dumps(asdict(current_location), sort_keys=True)
    result = client.publish(location_parsed_topic, msg, qos=1, retain=True)
    status = result[0]
    if status == 0:
        print(f"[{location_parsed_topic} send] `{msg}`")
    else:
        print(f"[{location_parsed_topic} send] FAIL")

def on_log(client, userdata, level, buf):
    print("[MQTT LOG] ",buf)

mqtt_service = None  # type: mqtt_client.Client


###  Flask HTTP server ###

flask_http_app = Flask("IoT PeTracker Cloud Map Service")

HTML_TEMPLATE = """<!DOCTYPE html>
<html>
    <head>
        <title> PeTracker</title>
        <meta http-equiv="refresh" content="10" >
    </head>
    <body>
        {}
    </body> 
</html>
"""

@flask_http_app.route('/')
def index():
    global current_location, distress
    coords = [current_location.latitude, current_location.longitude]
    folium_map = folium.Map(location=coords, zoom_start=18, zoom_control=False)
    popup_format = f"lat: {current_location.latitude}<br>" \
                   f"lon: {current_location.longitude}<br>" \
                   f"altitude: {current_location.altitude}<br>" \
                   f"utc time: {current_location.fix_time}<br>"\
                   f"Distress enabled: {distress}"
    folium.Marker(
        location=coords,
        popup=folium.Popup(popup_format, max_width=200, show=True),
        icon=folium.Icon(icon="fa-paw", prefix="fa"),
        tooltip="Your fluffy"
    ).add_to(folium_map)
    s = HTML_TEMPLATE.format(folium_map._repr_html_())
    return s

if __name__ == '__main__':
    mqtt_service = connect_mqtt()
    mqtt_service.on_log = on_log
    mqtt_service.loop_start()
    while not mqtt_is_connected:
        time.sleep(1)
    subscribe(mqtt_service)
    print("Waiting for subscription success")
    time.sleep(5)
    flask_http_app.run()
    mqtt_service.loop_stop(True)
