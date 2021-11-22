import sys
import os
import serial
import time
import signal 
import random

from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

import paho.mqtt.client as mqtt

# MQTT Value
TPE = ThreadPoolExecutor(max_workers=2)
PPE = ThreadPoolExecutor(max_workers=2)

broker = 'broker.emqx.io'
port = 1883
topic = "python/depth"

client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

# Motor Control Function
class MotorCon:

    def __init__(self):
        
        # Motor Control Value
        self.data = ''
        # self.fifoFileName = "/tmp/uart_fifo"
        # self.fifoMode = 0o777
        self.keyMap = {b'A' : 'turn left::1',
                        b'B' : 'turn left::2',
                        b'C' : 'turn left::3',
                        b'D' : 'turn left::4',
                        b'E' : 'turn right::4',
                        b'F' : 'turn right::3',
                        b'G' : 'turn right::2',
                        b'H' : 'turn right::1',
                        b'h' : 'forward',
                        b'k' : 'back',
                        b'j' : 'stop'}
        self.serialPort = serial.Serial()
        
    # def create_fifo(self):
        
    #     try:
    #         os.remove(self.fifoFileName)
    #     except FileNotFoundError:
    #         print("FIFO file is not found.")
    #         pass
    #     try:
    #         os.mkfifo(self.fifoFileName, self.fifoMode)
    #     except FileExistsError:
    #         print("FIFO file is exist.")    
    #         pass

    # openport
    def openSerial(self):
        #print("1")
        
        self.serialPort = serial.Serial(
            #port="/dev/ttyUSB0",
            port="/dev/ttyUSB1",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )

        return self.serialPort
    
    def txThread(self, arg):
        #print("2")
        
        self.data = arg
        self.serialPort.write(self.data.encode("utf-8"))
        print("input data : ", self.data)

        # if self.keyMap[self.data]:
            
        #     # with open(self.fifoFileName , 'w') as fw:
        #     #     fw.writelines(self.data + "\n")
        #     #     print("input data : ", self.data)

        #     self.serialPort.write(self.data.encode("utf-8"))
        #     print("input data : ", self.data)
        
        #     # with open(self.fifoFileName, "wb") as fifo:
        #     #     fifoData = fifo.write(self.data)
        #     #     print("data : ", fifoData)
        
        # else:
        #     print("Wrong data received from depth...")

    def rxThread(self):
        #print("3")
        
        # with open(self.fifoFileName , 'r') as fr:
        #     self.data = fr.read()
        #     print("output data : " + self.data)
        
        # self.serialPort.read()
        self.data = self.serialPort.readline()
        print("output data : " + self.data.decode("utf-8"))
           
    def cmd_function(self, arg):
        print("1")

        #self.create_fifo()
        # motorCon = MotorCon()
        # motorCon.openSerial()

        # TPE.submit(self.txThread, arg)
        # TPE.submit(self.rxThread)
        #TPE.shutdown()
        
        self.txThread(arg)
        self.rxThread()
        
# MQTT Function
def on_log(server, obj, level, string):
    print(f"log : {string}")
    if string == "Sending PINGREQ":
        server.disconnect(reasoncode=0)

def on_connect(server, userdata, flags, rc):
    print("connect result : " + str(rc))

    server.subscribe(topic)

def on_disconnect(server, userdata, flags, rc=0):
    print("sensor off : " + str(rc))

def on_message(server, userdata, msg):

    print(msg.topic + " " + str(msg.payload))
    arg_chk = msg.payload.decode("utf-8")
    print("arg : " + arg_chk)
    if arg_chk == b'q':
        print("quit command.")
        server.disconnect(reasoncode=0)
        sys.exit()

    PPE.map(motor.txThread, arg_chk)
    PPE.submit(motor.rxThread)

    # with ProcessPoolExecutor(max_workers=2) as PPE:
        
    #     PPE.map(MotorCon.cmd_function, arg_chk)
        
    #     try:
    #         PPE.shutdown(wait=True)
    #     except RuntimeError:
    #         print("process is alerady shutdowned -> Runtimeout.")

def on_subscribe(server, obj, mid, granted_qos):
    print("Subscribed : " + str(mid) + " " + str(granted_qos))

if __name__ == '__main__':

    motor = MotorCon()
    motor.openSerial()
    
    try:
        server = mqtt.Client(client_id)
        server.username_pw_set(username, password)
        
        
        connect_chk = server.loop_misc()
        if connect_chk == 7:
            print("publisher disconnected.")
            server.disconnect(reasoncode=0)

        else:    
            server.on_log = on_log
            server.on_message = on_message
            server.on_connect = on_connect
            server.on_disconnect = on_disconnect
            server.on_subscribe = on_subscribe
            server.connect(broker, port)
            # server.connect("mqtt.eclipseprojects.io", 1883, 60)
            # server.connect("test.mosquitto.org", 1883, 60)
            
            server.loop_forever()

            

    except KeyboardInterrupt:
        sys.exit()
