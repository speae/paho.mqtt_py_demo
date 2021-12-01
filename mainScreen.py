import sys
import os
import random
import time
from paho.mqtt import client as mqtt_client
from mqttPub import *

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QFont, QFontDatabase
from PyQt5.QtCore import Qt

import paho_mqtt_pub_sub_command_key as cmd
import cv2
#from skimage.io import imread

###############################################################################
# MQTT Setting

broker = 'broker.emqx.io'
broker2 = "mqtt.eclipseprojects.io"
port = 1883
topic = "python/mqtt"
topicOldVersion = "python/oldversion"
topic_cancle = "python/cancle"
topicKeyboard = "python/keyboardControll"
mappingStart = "python/mappingStart"

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'emqx'
password = 'public'

def connect_mqtt():

    def on_log(client, obj, level, string):
        print(f"log : {string}")
        if string == "Received PINGRESP":
            client.reconnect()
            
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic)
            
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_log = on_log
    client.on_connect = on_connect
    client.connect(broker, port)
    #client.connect(broker2, port)
    return client

def publish_Following(client):
    time.sleep(1)
    
    msg = "deepsort_on"
    result = client.publish(topic, msg, 0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def publish_Following_old(client):
    time.sleep(1)

    msg = "deepsort_on_old"
    result = client.publish(topic, msg, 0)
    
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

def Following_Cancle(client):
    time.sleep(1)

    msg = "deepsort_off"
    result = client.publish(topic_cancle, msg, 0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic_cancle}`")
    else:
        print(f"Failed to send message to topic {topic}")

def publish_Nav(client):
    time.sleep(1)
    
    msg = "nav_on"
    result = client.publish(topic, msg, 0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

# def publish_Mapping_Start(client):
#     time.sleep(1)
    
#     msg = "mapping_start_on"
#     #rospy.init_node('talker', anonymous=True)
#     result = client.publish(mappingStart, msg, 0)
#     # result: [0, 1]
#     status = result[0]
#     if status == 0:
#         print(f"Send `{msg}` to topic `{topic}`")
#     else:
#         print(f"Failed to send message to topic {topic}")

def publish_Map(client):
    os.system("rosrun map_server map_saver -f ~/map")
    # time.sleep(1)
    
    # msg = "map_save_on"
    # result = client.publish(topic, msg, 0)
    # # result: [0, 1]
    # status = result[0]
    # if status == 0:
    #     print(f"Send `{msg}` to topic `{topic}`")
    # else:
    #     print(f"Failed to send message to topic {topic}")

def publish_Command_Start(client):
    time.sleep(1)
    
    msg = "cmd_on"
    result = client.publish(topicKeyboard, msg, 0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")
        
def Nav_Cancle(client):
    time.sleep(1)
    
    msg = "nav_off"
    result = client.publish(topic_cancle, msg, 0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic_cancle}`")
    else:
        print(f"Failed to send message to topic {topic}")

###############################################################################
# PyQt Setting

# connect the ui file

# main screen design
Main_form_class = uic.loadUiType(
    "/home/nvidia/paho_mqtt_py_demo/ui/mainScreen.ui")[0]

# main screen design
Nav_form_class = uic.loadUiType(
    "/home/nvidia/paho_mqtt_py_demo/ui/Nav.ui")[0]

# following screen design
F_form_class = uic.loadUiType(
    "/home/nvidia/paho_mqtt_py_demo/ui/Following.ui")[0]

# following screen design
Voice_form_class = uic.loadUiType(
    "/home/nvidia/paho_mqtt_py_demo/ui/Voice.ui")[0]

# 화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, Main_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.showMaximized()
        self.Nav_Btn.clicked.connect(self.Nav_Btn_Function)
        self.F_Btn.clicked.connect(self.F_Btn_Function)
        # setting image to the button
        self.Voice_Btn.clicked.connect(self.Voice_Btn_Function)
        # setting image to the button
        #self.showMaximized()

    def Nav_Btn_Function(self):
        print("Nav_Btn Clicked (Nav_Btn Clicked!)")
        self.Nav_window()

    def F_Btn_Function(self):
        print("Following_Btn Clicked (Following_Btn Clicked!)")
        self.F_window()

    def Voice_Btn_Function(self):
        print("Voice_Btn Clicked (Voice_Btn Clicked!)")
        self.Voice_window()

    def Nav_window(self):
        self.n = Nav_WindowClass()
        self.n.show()
        self.hide()

    def F_window(self):
        self.f = F_WindowClass()
        self.f.show()
        self.hide()

    def Voice_window(self):
        self.v = Voice_WindowClass()
        self.v.show()
        self.hide()

class Nav_WindowClass(QMainWindow, Nav_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.showMaximized()
        self.Nav_Window_Cancle_Btn.clicked.connect(
            self.Nav_Window_Cancle_Btn_Function)
        self.Nav_Action_Btn.clicked.connect(
            self.Nav_Action_Btn_Function)
        # self.Mapping_Start_Btn.clicked.connect(
        #     self.Mapping_Start_Btn_Function)
        self.Map_Save_Btn.clicked.connect(
            self.Map_Save_Btn_Function)
        self.Command_Start_Btn.clicked.connect(
            self.Command_Start_Btn_Function)
        self.Show_Map_Btn.clicked.connect(
            self.Show_Map_Btn_Function)

        self.Nav_Action_Btn.setStyleSheet(
            "background-image : url(/home/nvidia/paho_mqtt_py_demo/img/map.png);"
            "background-position : center;")

    def Nav_Window_Cancle_Btn_Function(self):

        self.M_window()
        client = connect_mqtt()
        Nav_Cancle(client)
        
    def Nav_Action_Btn_Function(self):
        
        # print("Map_Create_Btn Clicked (Map_Create_Btn Clicked!)")
        # self.Map_window()
        client = connect_mqtt()
        publish_Nav(client)

    # def Mapping_Start_Btn_Function(self):
        
    #     # print("Map_Create_Btn Clicked (Map_Create_Btn Clicked!)")
    #     # self.Map_window()
    #     client = connect_mqtt()
    #     publish_Mapping_Start(client)

    def Map_Save_Btn_Function(self):
        
        # print("Map_Create_Btn Clicked (Map_Create_Btn Clicked!)")
        # self.Map_window()
        client = connect_mqtt()
        publish_Map(client)

    def Show_Map_Btn_Function(self):
        print("show me the map!!!")
        os.system("xdg-open ~/map.pgm")

    def Command_Start_Btn_Function(self):
        
        # print("Map_Create_Btn Clicked (Map_Create_Btn Clicked!)")
        self.Cmd_window()
        # client = connect_mqtt()
        # publish_Command_Start(client)

    def M_window(self):

        self.m = WindowClass()
        self.m.show()
        self.hide()

    def Cmd_window(self):
        self.c = cmd.Keyboard_WindowClass()
        self.c.show()

class F_WindowClass(QMainWindow, F_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.showMaximized()
        self.Following_Cancle_Btn.clicked.connect(
            self.Following_Cancle_Btn_Function)
        self.Following_Action_Btn.clicked.connect(
            self.Following_Action_Btn_Function)
        self.Following_Action_Old_Btn.clicked.connect(
            self.Following_Action_Old_Btn_Function)
            
        self.Following_Action_Btn.setStyleSheet(
            "background-image : url(/home/nvidia/paho_mqtt_py_demo/img/man4.png);"
            "background-position : center;")
        self.Following_Action_Old_Btn.setStyleSheet(
            "background-image : url(/home/nvidia/paho_mqtt_py_demo/img/man4.png);"
            "background-position : center;")

    def Following_Cancle_Btn_Function(self):
        self.m = WindowClass()
        self.m.show()
        self.hide()
        client = connect_mqtt()
        Following_Cancle(client)

    def Following_Action_Btn_Function(self):

        # client defenition
        # user setting
        # on_connect
        # connect
        client = connect_mqtt()

        # publish
        publish_Following(client)

    def Following_Action_Old_Btn_Function(self):

        # client defenition
        # user setting
        # on_connect
        # connect
        client = connect_mqtt()

        # publish
        publish_Following_old(client)

class Voice_WindowClass(QMainWindow, Voice_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.showMaximized()
        self.Voice_Cancle_Btn.clicked.connect(self.Voice_Cancle_Btn_Function)
        self.Voice_Action_Btn.clicked.connect(self.Voice_Action_Btn_Function)
        self.Voice_Action_Btn.setStyleSheet(
            "image : url(/home/nvidia/paho_mqtt_py_demo/img/mic.png);"
            "image-position: center;")

    def Voice_Cancle_Btn_Function(self):
        self.m = WindowClass()
        self.m.show()
        self.hide()

    def Voice_Action_Btn_Function(self):
        pass

# WindowClass -> F_WindowClass -> Following_Action_Btn_Function -> publish_Following  
if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.exec_()
    
