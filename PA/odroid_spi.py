import Odroid.GPIO as GPIO
import spidev
GPIO.setmode(GPIO.BCM)
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 4000000
GPIO.setup(104, GPIO.OUT)