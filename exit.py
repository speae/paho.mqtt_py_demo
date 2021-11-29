import random
import os
import sys
import psutil
import subprocess 
import signal
import serial

import rospy
from roslaunch import rlutil, parent
from paho.mqtt import client as mqtt_client


broker = 'broker.emqx.io'
port = 1883
topic_cancle = "python/cancle"
topicStop = "python/motorStop"
lidarStop = "python/lidarStop"

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

def kill_deepsort_pid():

    for proc in psutil.process_iter():
        try:
            processName = proc.name()
            processID = proc.pid

            if processName[:7] == "python3":
                commandLine = proc.cmdline()

                if "paho_mqtt_pub_depth_to_motor_test.py" in commandLine:
                    parent_pid = processID
                    parent = psutil.Process(parent_pid)

                    for child in parent.children(recursive=True):
                        child.kill()
                        
                    parent.kill()

                elif "track_tLose.py" in commandLine:
                    parent_pid = processID
                    parent = psutil.Process(parent_pid)

                    for child in parent.children(recursive=True):
                        child.kill()
                        
                    parent.kill()

                else:
                    print(processName, ' ', commandLine, ' - ', processID)
        
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            print("Can't find process ...")
            pass

def connect_mqtt() -> mqtt_client:

    def on_message(client, userdata, msg):
        
        str_msg = str(msg.payload.decode("utf-8"))
        #print(type(msg))
        if msg.retain:
            print("still alive message.")
            os.system(f"python3 retained-messages.py -b {broker} -u {username} -P{password} -t{topic_cancle} -p{port} -c")
        
        if str_msg == "deepsort_off":
            print(f"Quit received `{str_msg}` from `{msg.topic}` topic")
            
            serialPort = serial.Serial(
                port="/dev/ttyUSB1",
                #port="/dev/ttyUSB0",
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )   

            i = 0
            while i < 20: 
                serialPort.write(b'j')
                i += 1

            client.publish(topicStop, b'j')
            print("motor STOP...")

            kill_deepsort_pid()
            print("deepsort OFF..")
            
        elif str_msg == "nav_off":
            print(f"Quit received `{str_msg}` from `{msg.topic}` topic")
           
            client.publish(lidarStop, str_msg)
            print("navigation OFF..")

    def on_connect(client, userdata, flags, rc):
        
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic_cancle)
            
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
    client.loop_forever()

if __name__ == "__main__":
    run()