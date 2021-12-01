import mainScreen

# MQTT Setting
broker = 'broker.emqx.io'
port = 1883
topicKeyboard = "python/keyboardControll"

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{mainScreen.random.randint(0, 1000)}'
username = 'emqx'
password = 'public'

flag = 0
# Command_form_class = mainScreen.uic.loadUiType(
#     "/home/nvidia/paho_mqtt_py_demo/ui/command.ui")[0]

class Keyboard_WindowClass(mainScreen.QMainWindow): 
    def __init__(self): 
        super().__init__() 
        self.control = False

        myWidget = mainScreen.QWidget()
        self.setCentralWidget(myWidget)
        
        layout = mainScreen.QGridLayout()
        self.setLayout(layout)
 
        self.label = mainScreen.QLabel("Command result : ")
        layout.addWidget(self.label, 0, 0)

        self.resultLabel = mainScreen.QLabel()
        layout.addWidget(self.resultLabel, 0, 1)

        myWidget.setLayout(layout)
        
    def connect_mqtt(self):

        def on_log(client, obj, level, string):
            print(f"log : {string}")
            if string == "Received PINGRESP":
                client.reconnect()
        
        def on_connect(client, userdata, flags, rc):
            global flag
            if rc == 0:                                                                                      
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mainScreen.mqtt_client.Client(client_id)
        client.username_pw_set(username, password)
        client.on_log = on_log
        client.on_connect = on_connect
        client.connect(broker, port)
        return client

    def keyPressEvent(self, e): 
        
        client = self.connect_mqtt()
        
        if e.text() == chr(65) or e.text() == chr(97): 
            client.publish(topicKeyboard, b'A')
            self.resultLabel.setText("turn right:: ▶▶▶▶")
            
        elif e.text() == chr(66) or e.text() == chr(98): 
            client.publish(topicKeyboard, b'B')
            self.resultLabel.setText("turn right:: ▶▶▶")
            
        elif e.text() == chr(67) or e.text() == chr(99): 
            client.publish(topicKeyboard, b'C')
            self.resultLabel.setText("turn right:: ▶▶")
            
        elif e.text() == chr(68) or e.text() == chr(100): 
            client.publish(topicKeyboard, b'D')
            self.resultLabel.setText("turn right:: ▶")
            
        elif e.text() == chr(69) or e.text() == chr(101): 
            client.publish(topicKeyboard, b'E')
            self.resultLabel.setText("◀ ::turn left")
            
        elif e.text() == chr(70) or e.text() == chr(102): 
            client.publish(topicKeyboard, b'F')
            self.resultLabel.setText("◀◀ ::turn left")
            
        elif e.text() == chr(71) or e.text() == chr(103): 
            client.publish(topicKeyboard, b'G')
            self.resultLabel.setText("◀◀◀ ::turn left")
            
        elif e.text() == chr(72) or e.text() == chr(104): 
            client.publish(topicKeyboard, b'H')
            self.resultLabel.setText("◀◀◀◀ ::turn left")
            
        elif e.text() == chr(105): 
            client.publish(topicKeyboard, b'i')
            self.resultLabel.setText("▲")
            
        elif e.text() == chr(107): 
            client.publish(topicKeyboard, b'k')
            self.resultLabel.setText("▼")
            
        elif e.text() == chr(106): 
            client.publish(topicKeyboard, b'j')
            self.resultLabel.setText("■ STOP ■")

        elif e.text() == mainScreen.Qt.Key_Up: 
            client.publish(topicKeyboard, b'i')
            self.resultLabel.setText("▲")

        elif e.text() == mainScreen.Qt.Key_Down: 
            client.publish(topicKeyboard, b'k')
            self.resultLabel.setText("▼")

        elif e.text() == mainScreen.Qt.Key_Left: 
            client.publish(topicKeyboard, b'E')
            self.resultLabel.setText("◀")

        elif e.text() == mainScreen.Qt.Key_Right: 
            client.publish(topicKeyboard, b'D')
            self.resultLabel.setText("▶")    

        elif e.text() == mainScreen.Qt.Key_Escape: 
            client.publish(topicKeyboard, b'j')
            self.resultLabel.setText("■ STOP ■")

        else:
            print(f"{e.text()} :: retry.")  

if __name__ == '__main__': 
    app = mainScreen.QApplication(mainScreen.sys.argv)
    myWindow = Keyboard_WindowClass()
    myWindow.show()
    app.exec_()
