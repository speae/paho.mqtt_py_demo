import sys
import os
import threading
import time
import serial
import signal 
import random

import paho.mqtt.client as mqtt

# MQTT Value
broker = 'broker.emqx.io'
port = 1883
topic = "python/keyboard"

client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

# Motor Control Function
class MotorCon:

    def mainMenu(self):
        
        print("\n\n")
        print("-------------------------------------------------")
        print("                    MAIN MENU")
        print("-------------------------------------------------")
        print(" power volume : left 1 <<< 4 & right 4 >>> 1     ")
        print("-------------------------------------------------")
        
        print(" A. Turn Left1                                   ")
        print(" B. Turn Left2                                   ")
        print(" C. Turn Left3                                   ")
        print(" D. Turn Left4                                   ")
        print(" E. Turn Right4                                  ")
        print(" F. Turn Right3                                  ")
        print(" G. Turn Right2                                  ")
        print(" H. Turn Right1                                  ")
        print(" h. Forward                                      ")
        print(" k. Backward                                     ")
        print(" j. stop                                         ")

        print("-------------------------------------------------")
        print(" q. Motor Control application QUIT               ")
        print("-------------------------------------------------")
        print("\n\n")

        key = input("main menu select : ")

        return key

# MQTT Function
def on_log(client, obj, level, string):
    print(string)

def on_connect(client, userdata, flags, rc):
    print("connect result " + str(rc))

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    
def on_publish(client, obj, mid):
    print("mid : " + str(mid))

if __name__ == '__main__':

    try:
        client = mqtt.Client(client_id)
        client.username_pw_set(username, password)
        client.on_log = on_log
        client.on_message = on_message
        client.on_connect = on_connect
        client.on_publish = on_publish
         
        client.connect(broker, port)
        # client.connect("test.mosquitto.org", 1883, 60)

        motorCon = MotorCon()
        
        client.loop_start()
        while True:
            command = motorCon.mainMenu()
            if command == 'q':
                print("quit command.")
                client.disconnect(reasoncode=0)
            
            # pub_chk = client.publish("mqtt/paho", command)
            pub_chk = client.publish(topic, command)
            pub_chk.wait_for_publish()

    except KeyboardInterrupt:
        client.disconnect()
        sys.exit()
