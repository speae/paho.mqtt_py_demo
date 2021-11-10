import sys
import os
import threading
import time
import serial
import signal 
import argparse

from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

from rplidar import RPLidar
import paho.mqtt.client as mqtt

parser = argparse.ArgumentParser(description="select control sensor.")
parser.add_argument('--sensor', type=str, required=True, help="--sensor=<선택할 센서명: ㅣ(라이다), k(키보드), ...>")

args = parser.parse_args()

# RPLidar Control Function
class Lidar_Con:

    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSB0')
        
    def lidarInfo(self):
        info = self.lidar.get_info()
        print(info)
 
        health = self.lidar.get_health()
        print(health)

    def lidarScan(self):
        for i, scan in enumerate(self.lidar.iter_scans()):
            print('%d: Got %d measurments' % (i, len(scan)))
            return scan

    def scanData(self, quality, angle, distance):
        if quality == 0 or distance == 0.0:
            return 'i'
        elif quality > 0 and distance <= 500.0:
            return 'd'
        elif quality > 0 and distance > 500.0:
            return 'c'
                
    def lidarOFF(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

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

    config = {
        "sensor" : args.sensor
    }

    client = mqtt.Client()
    client.on_log = on_log
    client.on_message = on_message
    client.on_connect = on_connect
    client.on_publish = on_publish
        
    client.connect("mqtt.eclipseprojects.io", 1883, 60)
    #client.connect("test.mosquitto.org", 1883, 60)

    # automatic control mode
    if config["sensor"] == 'l':

        try:
    
            RP = Lidar_Con()
            # RP.lidarInfo()
            lidar = RPLidar('/dev/ttyUSB0')

            client.loop_start()
            for i, scan in enumerate(lidar.iter_scans()):
                print('%d: Got %d measurments' % (i, len(scan)))
                for data in zip(scan):
                    for quality, angle, distance in data:
                        print("quality : %d, angle : %f, distance : %f" % (quality, angle, distance))
                        result = RP.scanData(quality, angle, distance)

                        pub_chk = client.publish("mqtt/paho", result)
                        print("command : ", result)
                        pub_chk.wait_for_publish()

            # scanProcess = RP.lidarScan()
            
            # for order, scanResult in enumerate(scanProcess):
            #     command = RP.scanData(scanResult)
                
            #     pub_chk = client.publish("mqtt/paho", command)
            #     print("command : ", command)
            #     pub_chk.wait_for_publish()

        except KeyboardInterrupt:
            RP.lidarOFF()
            # lidar.stop()
            # lidar.stop_motor()
            # lidar.disconnect()
            # client.disconnect()
            sys.exit()

    # manual control mode
    elif config["sensor"] == 'k':

        try:
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
   