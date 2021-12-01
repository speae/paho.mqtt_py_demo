import random
import os

from paho.mqtt import client as mqtt_client

class mqtt_driver():

    def __init__(self):   
        self.broker = 'broker.emqx.io'
        self.broker2 = "mqtt.eclipseprojects.io"
        self.port = 1883
        self.topic = "python/mqtt"

        # generate client ID with pub prefix randomly
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        self.username = 'emqx'
        self.password = 'public'

    def connect_mqtt(self) -> mqtt_client:
        
        # def on_log(client, obj, level, string):
        #     print(f"log : {string}")
        #     if string == "Received PINGRESP":
        #         client.reconnect(reasoncode=0)

        def on_message(client, userdata, msg):
            str_msg = str(msg.payload.decode("utf-8"))
            #print(type(msg))
            if msg.retain:
                print("still alive message.")
                os.system(f"python3 retained-messages.py -b {self.broker} -u {self.username} -P{self.password} -t{self.topic} -p{self.port} -c")
            
            if str_msg == "deepsort_on":
                print(f"Received `{str_msg}` from `{msg.topic}` topic")
                print("deepsort ON...")
                os.system("python3 paho_mqtt_pub_depth_to_motor_test.py")

            elif str_msg == "deepsort_on_old":
                print(f"Received `{str_msg}` from `{msg.topic}` topic")
                print("deepsort ON... OLD version...........")
                os.system("python3 track_tLose.py")

            elif str_msg == "nav_on":
                print(f"Received `{str_msg}` from `{msg.topic}` topic")
                print("navigation ON...")
                os.system("python3 paho_mqtt_pub_sub_lidar.py")

            # elif str_msg == "mapping_start_on":
            #     print(f"Received `{str_msg}` from `{msg.topic}` topic")
            #     print("navigation ON...")
            #     os.system("python3 paho_mqtt_sub_mapping_start.py")    
        
        def on_subscribe(server, obj, mid, granted_qos):
            print("Subscribed : " + str(mid) + " " + str(granted_qos))

        def on_connect(client, userdata, flags, rc):
            global flag
            if rc == 0:                                                                                      
                print("Connected to MQTT Broker!")
                client.subscribe(self.topic)
                
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(self.client_id)
        client.username_pw_set(self.username, self.password)
        #client.on_log = on_log
        client.on_message = on_message
        client.on_subscribe = on_subscribe
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        #client.connect(self.broker2, self.port)
        return client

    def run(self):
        client = self.connect_mqtt()
        client.loop_forever()

if __name__ == '__main__':
    md = mqtt_driver()
    md.run()
