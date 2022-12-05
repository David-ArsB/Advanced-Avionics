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
#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
import os
from gps import *
from time import *
import time
import threading

gpsd = None #seting the global variable

os.system('clear') #clear the terminal (optional)

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread

  N = 0
  avg_lat = 0
  avg_long = 0

  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc

      os.system('clear')

      print()
      print (' GPS reading')
      print ('----------------------------------------')
      print ('latitude        ' , gpsd.fix.latitude)
      print ('longitude       ' , gpsd.fix.longitude)
      avg_lat = (avg_lat * N + gpsd.fix.latitude)/(N+1)
      avg_long = (avg_long * N + gpsd.fix.longitude)/(N+1)
      print('average latitude ', avg_lat)
      print('average longitude', avg_long)
      print ('time utc        ' , gpsd.utc,' + ', gpsd.fix.time)
      print ('altitude (m)    ' , gpsd.fix.altitude)
      print ('eps             ' , gpsd.fix.eps)
      print ('epx             ' , gpsd.fix.epx)
      print ('epv             ' , gpsd.fix.epv)
      print ('ept             ' , gpsd.fix.ept)
      print ('speed (m/s)     ' , gpsd.fix.speed)
      print ('climb           ' , gpsd.fix.climb)
      print ('track           ' , gpsd.fix.track)
      print ('mode            ' , gpsd.fix.mode)
      print()
      print('sats             ', gpsd.satellites)

      time.sleep(1)  # set to whatever
      N+=1

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("\nKilling Thread...")
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
  print("Done.\nExiting.")