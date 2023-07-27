import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import json
import time
import serial
import pynmea2


broker_address = ""
broker_port = 1883
client_id = ""

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("topic/log")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
TRIG = 3
ECHO = 17


def send_result(port, baudrate=9600, timeout=1):
    ser = serial.Serial(port, baudrate, timeout=timeout)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    GPIO.output(TRIG, False)

    try:
        while True:
            # micro
            GPIO.output(TRIG, True)
            time.sleep(1)
            GPIO.output(TRIG, False)

            while GPIO.input(ECHO) == 0:
                start = time.time()

            while GPIO.input(ECHO) == 1:
                stop = time.time()

            check_time = stop - start
            distance = check_time * 34300 / 2

            #neo-6m
            data = ser.readline().decode('utf-8')  # GPS 모듈로부터 데이터 읽기
            if data.startswith('$GPGGA'):  # GGA 문장 필터링
                msg = pynmea2.parse(data)
                latitude = msg.latitude  # 위도
                longitude = msg.longitude  # 경
                print("Distance : %.1f cm" % distance)
                print(data)
                client.publish("topic", json.dumps({"message":"7F10011001202023072113125700%.8f%.7f09500001%1.f007E"%(latitude,longitude,distance) }))
                time.sleep(1)
    except serial.SerialException as e:
            print(f"Error: {e}")
    except pynmea2.ParseError as e:
            print(f"Error parsing GPS data: {e}")
    
    except KeyboardInterrupt:
        print("complete")
        GPIO.cleanup()
        
client = mqtt.Client(client_id)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, broker_port, 60)

client.loop_start()

if __name__ == "__main__":
    serial_port = '/dev/serial0'  # 사용하는 시리얼 포트 이름 입력
    send_result(serial_port)