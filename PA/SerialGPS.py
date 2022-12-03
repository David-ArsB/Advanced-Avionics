# import serial
# import pynmea2
#
# port = "/dev/serial0"
#
#
# def parseGPS(string):
#     if string.find('GGA') > 0:
#         msg = pynmea2.parse(string)
#         print("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s -- Satellites: %s" % (
#         msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units, msg.num_sats))
#
#
# serialPort = serial.Serial(port, baudrate=9600, timeout=0.5)
# while True:
#     string = serialPort.readline()#.decode('UTF-8', errors='replace')
#     print(string)
#     parseGPS(string)
from gps import *
import time

gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
print('latitude\tlongitude\ttime utc\t\t\taltitude\tepv\tept\tspeed\tclimb')  # '\t' = TAB to try and output the data in columns.

try:

    while True:
        report = gpsd.next()  #
        if report['class'] == 'TPV':
            print(getattr(report, 'lat', 0.0), "\t",
                  getattr(report, 'lon', 0.0), "\t",
                  getattr(report, 'time', ''), "\t",
                  getattr(report, 'alt', 'nan'), "\t\t",
                  getattr(report, 'epv', 'nan'), "\t",
                  getattr(report, 'ept', 'nan'), "\t",
                  getattr(report, 'speed', 'nan'), "\t",
                  getattr(report, 'climb', 'nan'), "\t")

        time.sleep(1)

except (KeyboardInterrupt, SystemExit):  # when you press ctrl+c
    print("Done.\nExiting.")