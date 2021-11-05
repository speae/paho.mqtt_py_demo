import paho.mqtt.client as mqtt

mqtt = mqtt.Client("pub")

mqtt.connect("localhost", 1883)

try:

    while True:
        my_payload = input("payload : ")

        mqtt.publish("mqtt/paho", my_payload)

except KeyboardInterrupt:
    mqtt._clean_session