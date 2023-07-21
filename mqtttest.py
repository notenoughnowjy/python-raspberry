import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
import json

broker_address = ""
broker_port = 1883
client_id = ""
radar_pin = 18

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("test_topic/log")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

def radar_callback(channel):
    if(GPIO.input(radar_pin)):
        print("Motion detected")
        return {"message":"Motion detected"}
    else:
        print("Not detected")
        return {"message":"Not detected"}

def main():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(radar_pin, GPIO.IN)

    GPIO.add_event_detect(radar_pin, GPIO.BOTH, callback = radar_callback)

    try:
        print("CDM324 Detecting...")
        while True:
            message = radar_callback(channel=7)
            client.publish("test_topic", json.dumps(message))
            time.sleep(10)
    except KeyboardInterrupt:
        print("Exit program")
        client.disconnect()
    finally:
        GPIO.cleanup()



client = mqtt.Client(client_id)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, broker_port, 60)

main()