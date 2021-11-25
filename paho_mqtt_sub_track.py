import sys
import os
import random

import paho.mqtt.client as mqtt

broker = 'broker.emqx.io'
port = 1883

topicOldVersion = "python/oldversion"

client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'emqx'
password = 'public'

def on_connect(client, userdata, flags, rc):
    print("connect result " + str(rc))

    client.subscribe(topicOldVersion)

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    os.system("./RC_Control_fifo_hover")

if __name__ == '__main__':
    
    try:
        client = mqtt.Client(client_id)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.on_message = on_message

        # client.connect("localhost", 1883, 60)
        # client.connect("test.mosquitto.org", 1883, 60)
        client.connect(broker, port)

        client.loop_forever()

    except KeyboardInterrupt:
        sys.exit()