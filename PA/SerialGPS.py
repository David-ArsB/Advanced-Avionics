import serial
import pynmea2

port = "/dev/serial0"


def parseGPS(string):
    if string.find('GGA') > 0:
        msg = pynmea2.parse(string)
        print("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s -- Satellites: %s" % (
        msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units, msg.num_sats))


serialPort = serial.Serial(port, baudrate=9600, timeout=0.5)
while True:
    string = serialPort.readline().decode('ascii', errors='replace')
    print(string)
    parseGPS(string)