import mainScreen

# MQTT Setting
broker = 'broker.emqx.io'
port = 1883
topicKeyboard = "python/keyboardControll"

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{mainScreen.random.randint(0, 1000)}'
username = 'emqx'
password = 'public'

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
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mainScreen.mqtt_client.Client(client_id)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client

    def keyPressEvent(self, e): 
        
        client = self.connect_mqtt()
        inputKey = []
        if e.modifiers() & mainScreen.Qt.ShiftModifier: 
            inputKey.append(e.text())
            print(inputKey)
        
        if e.text() == chr(65) or e.text() == chr(97): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'A')

        elif e.text() == chr(66) or e.text() == chr(98): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'B')

        elif e.text() == chr(67) or e.text() == chr(99): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'C')
        
        elif e.text() == chr(68) or e.text() == chr(100): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'D')
        
        elif e.text() == chr(69) or e.text() == chr(101): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'E')
        
        elif e.text() == chr(70) or e.text() == chr(102): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'F')

        elif e.text() == chr(71) or e.text() == chr(103): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'G')
        
        elif e.text() == chr(72) or e.text() == chr(104): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'H')

        elif e.text() == chr(105): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'i')

        elif e.text() == chr(107): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'k')

        elif e.text() == chr(106): 
            self.resultLabel.setText(e.text())
            client.publish(topicKeyboard, b'j')
             
        else:
            print("retry.")    

if __name__ == '__main__': 
    app = mainScreen.QApplication(mainScreen.sys.argv)
    myWindow = Keyboard_WindowClass()
    myWindow.show()
    app.exec_()
