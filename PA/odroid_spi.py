# import Odroid.GPIO as GPIO
# import spidev
# GPIO.setmode(GPIO.BCM)
# spi = spidev.SpiDev()
# spi.open(0, 0)
# spi.max_speed_hz = 4000000
# GPIO.setup(16, GPIO.OUT)

import Odroid.GPIO as GPIO
import time

redLedGPIO=16
GPIO.setmode(GPIO.BCM)
GPIO.setup(redLedGPIO,GPIO.OUT)

while True:
        GPIO.output(redLedGPIO,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(redLedGPIO,GPIO.LOW)
        time.sleep(1)