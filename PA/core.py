import sys, os, smbus, time
import RPi.GPIO as GPIO  # import gpio
GPIO.setmode(GPIO.BCM)  # set the gpio mode
import spidev # import SPI
from BMP388 import BMP388 # Import Pressure sensor
from LSM6DSL import LSM6DSL # Import Accelerometer and Gyro Module
from LIS3MDL import LIS3MDL # Import Compass module
from lib_nrf24 import NRF24 # Import Radio module
from GPSPoller import GpsPoller
from math import pi, atan2


class corePrimaryAircraft():
    RADIO_PAYLOAD_SIZE = 32
    RADIO_DATA_RATES = [NRF24.BR_250KBPS, NRF24.BR_1MBPS, NRF24.BR_2MBPS]
    RADIO_PA_LEVELS = [NRF24.PA_MIN, NRF24.PA_LOW, NRF24.PA_HIGH, NRF24.PA_MAX]

    # set the pipe addresses. this address should be entered on the receiver also
    RADIO_WRITING_PIPE = [0xE0, 0xE0, 0xE0, 0xE0, 0xE0]
    RADIO_READING_PIPE = [0xF0, 0xF0, 0xF0, 0xF0, 0xF0]

    def __init__(self):
        self.altimeter = BMP388(smbus.SMBus(0x01))
        self.imu = LSM6DSL(smbus.SMBus(0x01))
        self.compass = LIS3MDL(smbus.SMBus(0x01))
        self.radio = NRF24(GPIO, spidev.SpiDev())
        self.gps = GpsPoller()

        self._initRadio()
        self._initAltimeter()
        self._initCompass()
        self._initIMU()
        self._initGPS()
        print('Initialisation Complete! \n')


    def _initAltimeter(self):
        print('Setting up altimeter (BMP388) ...')
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        self.altimeter.setGroundPressure(pressure)
        print(' -> Ground Pressure Level = %.2f Pa' % (pressure / 100.0))

    def _initIMU(self):
        print('Setting up IMU (LSM6DSL) ...')

    def _initCompass(self):
        print('Setting up compass (LIS3MDL) ...')
        # Compensation and calibration?

    def _initRadio(self):
        print('Setting up radio (nRF24L01+) ...')
        radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins

        radio.begin(0, 25)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25
        radio.setPayloadSize(self.RADIO_PAYLOAD_SIZE)  # set the payload size as 32 bytes
        radio.setChannel(0x76)  # set the channel as 76 hex
        radio.setDataRate(self.RADIO_DATA_RATES[2])  # set radio data rate to 2MBPS
        radio.setPALevel(self.RADIO_PA_LEVELS[1])  # set PA level to LOW

        radio.setAutoAck(True)  # set acknowledgement as true
        radio.enableDynamicPayloads()
        radio.enableAckPayload()

        radio.openWritingPipe(self.RADIO_WRITING_PIPE)  # open the defined pipe for writing
        radio.openReadingPipe(0, self.RADIO_READING_PIPE)  # open the defined pipe for reading

    def _initGPS(self):
        print('Setting up GPS thread ...')
        self.gps.start()

    def printDataSummary(self):
        # Print Altimeter Data
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()

        print(' Temperature = %.1f Pressure = %.2f  Altitude =%.2f ' % (
            temperature / 100.0, pressure / 100.0, altitude / 100.0))
        # Print Compass Data
        magX = self.compass.readMAGx()
        magY = self.compass.readMAGy()
        magZ = self.compass.readMAGz()
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360
        print(' magX = %.2f magY = %.2f  magZ =%.2f ' % (magX, magY, magZ))
        print(' Heading = %.2f\n' % (heading))
        # Print IMU Data
        AccX = self.imu.readACCx()
        AccY = self.imu.readACCy()
        AccZ = self.imu.readACCz()
        GyrX = self.imu.readGYRx()
        GyrY = self.imu.readGYRy()
        GyrZ = self.imu.readGYRz()
        print('AccX = %.2f g\nAccY = %.2f g\nAccZ = %.2f g\n' % (AccX, AccY, AccZ))
        print('GyrX = %.2f dps\nGyrY = %.2f dps\nGyrZ = %.2f dps\n' % (GyrX, GyrY, GyrZ))
        # Print GPS Data
        lat, long = self.gps.getPosition()
        print('Latitude = %.8f, Longitude = %.8f \n' % (lat, long))

if __name__ == '__main__':
    core = corePrimaryAircraft()

    while True:
        core.printDataSummary()
        time.sleep(1)