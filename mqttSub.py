import random
import os
from paho.mqtt import client as mqtt_client


broker = 'broker.emqx.io'
port = 1883
topic = "python/mqtt"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        str_msg = str(msg.payload.decode("utf-8"))
        print(f"Received `{str_msg}` from `{msg.topic}` topic")
        #print(type(msg))
        print("deepsort ON")
        if str_msg == "deepsort_on":
            os.system('python3 paho_mqtt_sub_depth_test.py --source 2 --yolo_weight yolov5s.pt --show-vid --class 0')
        # if str_msg == "deepsort_off":
        #     os.system('pkill -f paho_mqtt_sub_depth_test.py')
        
    client.subscribe(topic)
    client.on_message = on_message

    # if on_message == "deepsort_on":
    #     #os.system('python track.py')
    # if on_message == "deepsort_off":
    #     print("deepsort OFF")
    #     os.system("pkill -f track.py")


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
