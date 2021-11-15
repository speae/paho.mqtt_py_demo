import sys
import os
import threading
import time
import serial
import signal 

import paho.mqtt.client as mqtt

# MQTT Value


# Motor Control Function
class MotorCon:

    def mainMenu(self):
        
        print("\n\n")
        print("-------------------------------------------------")
        print("                    MAIN MENU")
        print("-------------------------------------------------")
        print(" 1. apple                                        ")
        print(" 2. person(banana)                               ")
        print(" 3. bicycle                                      ")
        print(" 4. dog                                          ")
        print(" 5. truck                                        ")

        print(" a. Turn Left                                    ")
        print(" b. Turn Right                                   ")
        print(" c. Forward                                      ")
        print(" d. backward                                     ")
        print(" i. stop                                         ")
        print(" I. speed up +10                                 ")
        print(" D. speed down -10                               ")

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
        client = mqtt.Client()
        client.on_log = on_log
        client.on_message = on_message
        client.on_connect = on_connect
        client.on_publish = on_publish
         
        client.connect("mqtt.eclipseprojects.io", 1883, 60)
        # client.connect("test.mosquitto.org", 1883, 60)

        motorCon = MotorCon()
        
        client.loop_start()
        while True:
            command = motorCon.mainMenu()
            if command == 'q':
                client.disconnect()
                sys.exit()

            pub_chk = client.publish("mqtt/paho", command)
            pub_chk.wait_for_publish()

    except KeyboardInterrupt:
        client.disconnect()
        sys.exit()
