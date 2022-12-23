import sys, os, smbus
import RPi.GPIO as GPIO  # import gpio
GPIO.setmode(GPIO.BCM)  # set the gpio mode
import spidev # import SPI
from BMP388 import BMP388 # Import Pressure sensor
from LSM6DSL import LSM6DSL # Import Accelerometer and Gyro Module
from LIS3MDL import LIS3MDL # Import Compass module
from lib_nrf24 import NRF24 # Import Radio module
from GPSPoller import GpsPoller


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
        # Print Compass Data
        # Print IMU Data
        # Print GPS Data
        lat, long = self.gps.getPosition()
        print('Latitude = %.8f, Longitude = %.8f \n' % (lat, long))

if __name__ == '__main__':
    core = corePrimaryAircraft()

    while True:
        core.printDataSummary()