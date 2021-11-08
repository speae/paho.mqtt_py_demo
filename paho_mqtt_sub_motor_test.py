import sys
import os
import concurrent.futures
import threading
import time
import serial
import signal 

from multiprocessing import Pool, Process, Value, Array, TimeoutError
import paho.mqtt.client as mqtt

# MQTT Value
TPE = concurrent.futures.ThreadPoolExecutor(max_workers=6)

# Motor Control Function
class Motor_Con:

    def __init__(self):
        
        # Motor Control Value
        self.exitThread = False
        self.data = ''
        self.fifoFileName = "/tmp/uart_fifo"
        self.fifoMode = 0o777
        self.keyMap = {'a' : 'turn left',
                        'b' : 'turn right',
                        'c' : 'forward',
                        'd' : 'back',
                        'i' : 'stop'}
        self.serialPort = serial.Serial()
        
    def create_fifo(self):
        
        try:
            os.remove(self.fifoFileName)
        except FileNotFoundError:
            print("FIFO file is not found.")
            pass
        try:
            os.mkfifo(self.fifoFileName, self.fifoMode)
        except FileExistsError:
            print("FIFO file is exist.")    
            pass

    # openport
    def openSerial(self):
        #print("1")
        
        self.serialPort = serial.Serial(
            port="/dev/ttyTHS2",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )

        return self.serialPort
    
    def txThread(self, arg):
        #print("2")
        
        self.data = arg
        
        if self.keyMap[self.data]:  
            
            with open(self.fifoFileName , 'w') as fw:
                fw.writelines(self.data + "\n")
                print("input data : ", self.data)

            # self.serialPort.write(self.data.encode("utf-8"))
            # print("input data : ", self.data)
        
            # with open(self.fifoFileName, "wb") as fifo:
            #     fifoData = fifo.write(self.data)
            #     print("data : ", fifoData)

        else:
            self.exitThread = True
        
    def rxThread(self):
        #print("3")
        
        with open(self.fifoFileName , 'r') as fr:
            self.data = fr.read()
            print("output data : " + self.data)
        
        # self.serialPort.read()
        # self.data = self.serialPort.readline()
        # print("output data : " + self.data)
           
    def cmd_function(self, arg):
    
        self.create_fifo()
        self.openSerial()
        
        TPE.submit(self.txThread, arg)
        TPE.submit(self.rxThread)
        
        TPE.shutdown()
        
# MQTT Function
def on_log(server, obj, level, string):
    print(string)

def on_connect(server, userdata, flags, rc):
    print("connect result " + str(rc))

    server.subscribe("mqtt/paho")

def on_message(server, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    arg_chk = msg.payload.decode("utf-8")
    print("arg : " + arg_chk)
    if arg_chk == 'q':
        print("quit cammand.")
        server.disconnect()
        sys.exit()
    
    with Pool(processes=2) as pool:
        motorCon = Motor_Con()
    
        pool.apply_async(motorCon.cmd_function, (arg_chk,))
        try:
            pool.close()
            pool.join()
        except TimeoutError:
            print("We lacked patience and got a multiprocessing -> TimeoutError.")

def on_subscribe(server, obj, mid, granted_qos):
    print("Subscribed : " + str(mid) + " " + str(granted_qos))

if __name__ == '__main__':

    try:
        server = mqtt.Client()
        server.on_log = on_log
        server.on_message = on_message
        server.on_connect = on_connect
        server.on_subscribe = on_subscribe
        
        server.connect("mqtt.eclipseprojects.io", 1883, 60)
        # server.connect("test.mosquitto.org", 1883, 60)
        
        server.loop_forever()

    except KeyboardInterrupt:
        exitThread = True
