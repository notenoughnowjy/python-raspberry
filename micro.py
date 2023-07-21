import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import json
import time

broker_address = ""
broker_port = 1883
client_id = ""

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("test_topic/log")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIG = 3
ECHO = 17
print("micro_disance")

def micro():
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    GPIO.output(TRIG, False)
    print("micro output reset")

    try:
        while True:
            GPIO.output(TRIG, True)
            time.sleep(1)
            GPIO.output(TRIG, False)

            while GPIO.input(ECHO) == 0:
                start = time.time()

            while GPIO.input(ECHO) == 1:
                stop = time.time()

            check_time = stop - start
            distance = check_time * 34300 / 2
            print("Distance : %.1f cm" % distance)
            client.publish("test_topic", json.dumps({"message":"7F1001100120202307211312570036.56829211128.709214209500001%1.f007E" % distance }))
            time.sleep(1)

    except KeyboardInterrupt:
        print("complete")
        GPIO.cleanup()
        
client = mqtt.Client(client_id)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, broker_port, 60)

client.loop_start()

micro()