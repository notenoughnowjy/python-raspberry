import serial
import pynmea2


def read_gps_data():
    port = "/dev/ttyAMA0"
    baud_rate = 9600
    ser = serial.Serial(port, baud_rate, timeout=0.5)
    try:
        while True:
            data = ser.readline().decode("utf-8")
            if data.startswith("$GPGGA"):
                msg = pynmea2.parse(data)
                latitude = msg.latitude
                longitude = msg.longitude
                altitude = msg.altitude
                print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} meters")
    except KeyboardInterrupt:
        ser.close()
        print("GPS reading stopped.")

if __name__ == "__main__":
    read_gps_data()