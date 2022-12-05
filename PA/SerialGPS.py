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
import plotext as plt;

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
  done = False
  x = []
  y = []
  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
      if N == 10 and not done:
        N = 0
        avg_lat = 0
        avg_long = 0
        done = True
        x = []
        y = []

      os.system('clear')
      lat = gpsd.fix.latitude
      long = gpsd.fix.longitude
      print()
      print (' GPS reading')
      print ('----------------------------------------')
      print ('latitude        ' , lat)
      print ('longitude       ' , long)
      avg_lat = (avg_lat * N + lat)/(N+1)
      avg_long = (avg_long * N + long)/(N+1)
      print('average coords   ' , avg_lat,',',avg_long)
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
      y.append(lat)
      x.append(long)

      time.sleep(1)  # set to whatever
      N+=1

  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print("\nKilling Thread...")
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing



  l, p = avg_long, avg_lat;
  plt.plot(x,y, marker = 'x');
  #plt.plotsize(100, 30);
  plt.title('Some Smart Title');
  plt.xlabel('Longitude');
  plt.ylabel('Latitude');
  plt.ticks_color('red');
  plt.ticks_style('bold');
  #plt.xlim(-l // 10, l + l // 10);
  #plt.ylim(-1.5, 1.5);
  #xticks = [l * i / (2 * p) for i in range(2 * p + 1)];
  #xlabels = [str(i) + 'π' for i in range(2 * p + 1)];
  #plt.xticks(xticks, xlabels);
  plt.show()

  print("Done.\nExiting.")