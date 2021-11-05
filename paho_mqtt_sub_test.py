import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print("connect result " + str(rc))

    client.subscribe("mqtt/paho")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# client.connect("localhost", 1883, 60)
# client.connect("test.mosquitto.org", 1883, 60)
client.connect("mqtt.eclipseprojects.io", 1883)

client.loop_forever()
