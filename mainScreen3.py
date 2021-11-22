import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QFont, QFontDatabase
from PyQt5.QtCore import Qt

import random
import time
from paho.mqtt import client as mqtt_client
from mqttPub import *

###############################################################################
# MQTT Setting
broker = 'broker.emqx.io'
port = 1883
topic = "python/mqtt"
topic_cancle = "python/cancle"

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'emqx'
password = 'public'


def connect_mqtt():
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

def publish_Cancle(client):
    time.sleep(1)
    msg = "deepsort_off"

    result = client.publish(topic_cancle, msg, 0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic_cancle}`")
    else:
        print(f"Failed to send message to topic {topic}")

# def run():
#     client = connect_mqtt()
#     publish_Following(client)
#     publish_Cancle(client)

###############################################################################
# PyQt Setting

# connect the ui file

# main screen design
Main_form_class = uic.loadUiType(
    "/home/nvidia/python-GUI/ui/mainScreen.ui")[0]

# main screen design
Nav_form_class = uic.loadUiType(
    "/home/nvidia/python-GUI/ui/Nav.ui")[0]

# following screen design
F_form_class = uic.loadUiType(
    "/home/nvidia/python-GUI/ui/Following.ui")[0]

# following screen design
Voice_form_class = uic.loadUiType(
    "/home/nvidia/python-GUI/ui/Voice.ui")[0]

# 화면을 띄우는데 사용되는 Class 선언


class WindowClass(QMainWindow, Main_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.Nav_Btn.clicked.connect(self.Nav_Btn_Function)
        self.F_Btn.clicked.connect(self.F_Btn_Function)
        # setting image to the button
        self.Voice_Btn.clicked.connect(self.Voice_Btn_Function)
        # setting image to the button

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
        self.Nav_Window_Cancle_Btn.clicked.connect(
            self.Nav_Window_Cancle_Btn_Function)

    def Nav_Window_Cancle_Btn_Function(self):
        print("Nav_Window_Cancle_Btn Clicked (Nav_Window_Cancle_Btn Clicked!)")
        self.M_window()

    def M_window(self):
        self.m = WindowClass()
        self.m.show()
        self.hide()


class F_WindowClass(QMainWindow, F_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.Following_Cancle_Btn.clicked.connect(
            self.Following_Cancle_Btn_Function)
        self.Following_Action_Btn.clicked.connect(
            self.Following_Action_Btn_Function)
        self.Following_Action_Btn.setStyleSheet(
            "background-image : url(/home/nvidia/python-GUI/img/man4.png);"
            "background-position : center;")

    def Following_Cancle_Btn_Function(self):
        self.m = WindowClass()
        self.m.show()
        self.hide()
        client = connect_mqtt()
        publish_Cancle(client)

    def Following_Action_Btn_Function(self):

        # client defenition
        # user setting
        # on_connect
        # connect
        client = connect_mqtt()

        # publish
        publish_Following(client)

        # mqttPub.py
        # run()


class Voice_WindowClass(QMainWindow, Voice_form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.Voice_Cancle_Btn.clicked.connect(self.Voice_Cancle_Btn_Function)
        self.Voice_Action_Btn.clicked.connect(self.Voice_Action_Btn_Function)
        self.Voice_Action_Btn.setStyleSheet(
            "image : url(/home/nvidia/python-GUI/img/mic.png);"
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
