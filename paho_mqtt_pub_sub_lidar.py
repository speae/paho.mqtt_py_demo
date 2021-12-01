import os
import random

from roslaunch import rlutil, parent
from paho.mqtt import client as mqtt_client

client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

broker = 'broker.emqx.io'
port = 1883
lidarStop = "python/lidarStop"

flag = 0

uuid = rlutil.get_or_generate_uuid(None, False)
lidarStartArgs = ["rplidar_ros", "rplidar.launch"]
lidarMappingArgs = ["hector_slam_launch", "tutorial.launch"]
roslaunchLidarStart = rlutil.resolve_launch_arguments(lidarStartArgs)[0]
roslaunchLidarSlam = rlutil.resolve_launch_arguments(lidarMappingArgs)[0]
roslaunchLidarFiles = [roslaunchLidarStart, roslaunchLidarSlam]
lidarParent = parent.ROSLaunchParent(uuid, roslaunchLidarFiles) 

# lidarStartArgs = ["rplidar_ros", "rplidar.launch"]
# roslaunchLidarStart = rlutil.resolve_launch_arguments(lidarStartArgs)[0]
# roslaunchLidarFiles = [roslaunchLidarStart]
# lidarParent = parent.ROSLaunchParent(uuid, roslaunchLidarFiles) 

def connect_mqtt() -> mqtt_client:
    
    def on_message(client, userdata, msg):

        str_msg = str(msg.payload.decode("utf-8"))
        #print(type(msg))
        if msg.retain:
            print("still alive message.")
            os.system(f"python3 retained-messages.py -b {broker} -u {username} -P{password} -t{lidarStop} -p{port} -c")
        
        if str_msg == "nav_off":
            print(f"Received `{str_msg}` from `{msg.topic}` topic")
            
            lidarParent.shutdown()
            client.disconnect(reasoncode=0)
            print("navigation OFF...")
           
    def on_connect(client, userdata, flags, rc):

        if rc == 0:                                                                                      
            print("Connected to MQTT Broker!")
            client.subscribe(lidarStop)

        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_message = on_message
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def run():
    client = connect_mqtt()
    lidarParent.start()
    client.loop_forever()

if __name__ == '__main__':
    run()
