from gps import *
from time import *
import time
import threading, os


class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true

  def run(self):
    while self.running:
      self.gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

  def getPosition(self):
      return [self.gpsd.fix.latitude, self.gpsd.fix.longitude]

if __name__ == '__main__':
    gpsp = GpsPoller() # create the thread
    gpsp.start()

    while True:
        os.system('clear')
        lat = gpsp.gpsd.fix.latitude
        long = gpsp.gpsd.fix.longitude
        print()
        print(' GPS reading')
        print('----------------------------------------')
        print('latitude        ', lat)
        print('longitude       ', long)
        print('time utc        ', gpsp.gpsd.utc, ' + ', gpsp.gpsd.fix.time)
        print('altitude (m)    ', gpsp.gpsd.fix.altitude)
        print('eps             ', gpsp.gpsd.fix.eps)
        print('epx             ', gpsp.gpsd.fix.epx)
        print('epv             ', gpsp.gpsd.fix.epv)
        print('ept             ', gpsp.gpsd.fix.ept)
        print('speed (m/s)     ', gpsp.gpsd.fix.speed)
        print('climb           ', gpsp.gpsd.fix.climb)
        print('track           ', gpsp.gpsd.fix.track)
        print('mode            ', gpsp.gpsd.fix.mode)
        print()
        print('sats             ', gpsp.gpsd.satellites)