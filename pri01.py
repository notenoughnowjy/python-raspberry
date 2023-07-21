import RPi.GPIO as GPIO
import time as t

radar_pin = 18

def radar_callback(channel):
    if(GPIO.input(radar_pin)):
        print("Motion detected")
    else:
        print("Not detected")

def main():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(radar_pin, GPIO.IN)

    GPIO.add_event_detect(radar_pin, GPIO.BOTH, callback = radar_callback)

    try:
        print("CDM324 Detecting...")
        while True:
            t.sleep(1)
    except KeyboardInterrupt:
        print("Exit program")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()