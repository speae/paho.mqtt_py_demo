import sys
import os
import serial
import time
import signal 
import random

from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

import paho.mqtt.client as mqtt

# == ./RC_Control_fifo

# Motor Control Function
class MotorCon():

    def __init__(self):
    
        # Motor Control Value   
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
                        b'i' : 'forward',
                        b'k' : 'backward',
                        b'j' : 'stop'}
        
        self.serialPort = serial.Serial(
            port="/dev/ttyUSB0",
            #port="/dev/ttyUSB1",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )

    def txThread(self, data):
        
        self.serialPort.write(data)
        print(f"input data : {str(data)}")


    def rxThread(self):
        
        result = self.serialPort.readline()
        result = result.decode("utf-8")
        print(f"output data : {result}")

# MQTT Function
class mqttClass(MotorCon):

    def __init__(self):
        
        self.data = b''
        self.MC = MotorCon()
        self.keyMap = self.MC.keyMap
        
        # generate client ID with pub prefix randomly
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        self.username = 'emqx'
        self.password = 'public'
        
        # --> MQTT value
        self.server = mqtt.Client(self.client_id)
        self.broker = 'broker.emqx.io'
        self.port = 1883
        # self.topic = "python/keyboard"
        self.topic = "python/depth"
        self.topicStop = "python/motorStop"
        self.topicKeyboard = "python/keyboardControll"

    def connect_mqtt(self) -> mqtt:
        
        def on_log(server, obj, level, string):
            print(f"log : {string}")
            if string == "Received PINGRESP":
                server.reconnect()

        def on_message(server, userdata, msg):
            print(msg.topic + " " + str(msg.payload))
            stopCommand = [msg.topic, str(msg.payload)]
            try:
                if stopCommand[0] == self.topicStop and stopCommand[1] == b'j':
                    i = 0
                    while i < 20:
                        self.MC.txThread(b'j')
                        self.MC.rxThread()
                        i += 1
                    print("종료 버튼을 눌렀습니다. Motor Off...")                         
                    server.disconnect(reasoncode=0)

                self.data = msg.payload
                print("arg : " + self.data.decode("utf-8"))
            
                if self.keyMap[self.data]:
                    if self.data == b'q':
                        print("quit command.")
                        server.disconnect(reasoncode=0)
                    
                    with ProcessPoolExecutor(max_workers=2) as PPE:
                        PPE.submit(self.MC.txThread, self.data)
                        PPE.submit(self.MC.rxThread)
                        # PPE.map(motor.txThread, arg_chk)
                        # PPE.submit(motor.rxThread)
                        try:
                            PPE.shutdown(wait=True)
                        except RuntimeError:
                            print("process is alerady shutdowned -> Runtimeout.")
                            
            except KeyError:
                print("Wrong data received from depth...")
                pass

        def on_connect(server, userdata, flags, rc):
            print("connect result : " + str(rc))
            if rc == 0:
                print("server has subscribed.")
                server.subscribe(self.topic)
                server.subscribe(self.topicStop)
                server.subscribe(self.topicKeyboard)

            else:
                print("connect failed.")
                server.disconnect(reasoncode=0)    
                
        def on_disconnect(server, userdata, flags, rc=0):
            print("sensor off : " + str(rc))
            
        def on_subscribe(server, obj, mid, granted_qos):
            print("Subscribed : " + str(mid) + " " + str(granted_qos))

        self.server.username_pw_set(self.username, self.password)
        self.server.on_log = on_log
        self.server.on_message = on_message
        self.server.on_subscribe = on_subscribe
        self.server.on_connect = on_connect
        self.server.on_disconnect = on_disconnect
        self.server.connect(self.broker, self.port)
        return self.server        

    def run(self):
        server = self.connect_mqtt()
        server.loop_forever()
        
if __name__ == '__main__':
    
    try:
        mc = mqttClass()
        mc.run()
        # server.connect("mqtt.eclipseprojects.io", 1883, 60)
        # server.connect("test.mosquitto.org", 1883, 60)         
 
    except KeyboardInterrupt:
        sys.exit()
