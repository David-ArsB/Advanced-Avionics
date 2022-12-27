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

        self.altimeter.setGroundPressure(pressure / 100.0)
        print(' -> Ground Pressure Level = %.2f Pa' % (pressure / 100.0))

    def _initIMU(self):
        print('Setting up IMU (LSM6DSL) ...')

    def _initCompass(self):
        print('Setting up compass (LIS3MDL) ...')
        # Compensation and calibration?

    def _initRadio(self):
        print('Setting up radio (nRF24L01+) ...')
        self.radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins
        self.radio.begin(0, 25)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25

        self.radio.setPayloadSize(self.RADIO_PAYLOAD_SIZE)  # set the payload size as 32 bytes
        self.radio.setChannel(0x76)  # set the channel as 76 hex
        self.radio.setDataRate(self.RADIO_DATA_RATES[2])  # set radio data rate to 2MBPS
        self.radio.setPALevel(self.RADIO_PA_LEVELS[1])  # set PA level to LOW

        self.radio.setAutoAck(True)  # set acknowledgement as true
        self.radio.enableDynamicPayloads()
        self.radio.enableAckPayload()

        self.radio.openWritingPipe(self.RADIO_WRITING_PIPE)  # open the defined pipe for writing
        self.radio.openReadingPipe(0, self.RADIO_READING_PIPE)  # open the defined pipe for reading

    def _initGPS(self):
        print('Setting up GPS thread ...')
        self.gps.start()

    def printDataSummary(self):
        print('===============================================\n$ ')
        # Print Altimeter Data
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        print('$ ALTIMETER DATA:')
        print('$  -> Temperature = %.1f C\n$  -> Pressure = %.2f Pa\n$  -> Altitude =%.2f m\n$ ' % (
            temperature / 100.0, pressure / 100.0, altitude / 100.0))
        # Print Compass Data
        magX = self.compass.readMAGxCorr()
        magY = self.compass.readMAGyCorr()
        magZ = self.compass.readMAGzCorr()
        #print((magX**2+magY**2+magZ**2)**(1/2))
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360
        print('$ COMPASS DATA:')
        print('$  -> magX = %.2f G, magY = %.2f G, magZ =%.2f G' % (magX, magY, magZ))
        print('$  -> Heading = %.2f deg N\n$ ' % (heading))
        # Print IMU Data
        AccX = self.imu.readACCx()
        AccY = self.imu.readACCy()
        AccZ = self.imu.readACCz()
        GyrX = self.imu.readGYRx()
        GyrY = self.imu.readGYRy()
        GyrZ = self.imu.readGYRz()
        print('$ IMU DATA:')
        print('$  -> AccX = %.2f g\n$  -> AccY = %.2f g\n$  -> AccZ = %.2f g\n$ ' % (AccX, AccY, AccZ))
        print('$  -> GyrX = %.2f dps\n$  -> GyrY = %.2f dps\n$  -> GyrZ = %.2f dps\n$ ' % (GyrX, GyrY, GyrZ))
        # Print GPS Data
        lat, long = self.gps.getPosition()
        print('$ GPS DATA:')
        print('$  -> Latitude = %.8f N, Longitude = %.8f E\n$ ' % (lat, long))
        print('===============================================\n')

    def transmitToGCS(self):
        '''
        Transmits sensor data to ground station.

        The nRF24L01+ module can only send a maximum of 32 bytes at a time (i.e one block).
        Sensor information must be split in blocks of 32 bytes.
        Header block sends information concerning message structure and contents.

        :return: None
        '''
        print('Transmitting to ground station...')

        # Print Altimeter Data
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        # Print Compass Data
        magX = self.compass.readMAGxCorr()
        magY = self.compass.readMAGyCorr()
        magZ = self.compass.readMAGzCorr()
        # print((magX**2+magY**2+magZ**2)**(1/2))
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360
        # Print IMU Data
        AccX = self.imu.readACCx()
        AccY = self.imu.readACCy()
        AccZ = self.imu.readACCz()
        GyrX = self.imu.readGYRx()
        GyrY = self.imu.readGYRy()
        GyrZ = self.imu.readGYRz()
        # Print GPS Data
        lat, long = self.gps.getPosition()

        numBlocks = 8
        header = list('$b'+ str(int(numBlocks)) + ',tph' + ',lat' + ',long')
        block1 = list("temperature: %.1f" % round(temperature/100, 1))
        block2 = list("pressure: %.1f" % round(pressure/100, 1))
        block3 = list("altitude: %.1f" % round(altitude/100, 1))
        block4 = list("pos:" + str(lat) + ',' + str(long))
        block5 = list("Acc: %.1f,%.1f,%.1f" % (round(AccX, 1),round(AccY, 1),round(AccZ, 1)))
        block6 = list("Gyr: %.1f,%.1f,%.1f" % (round(GyrX, 1), round(GyrY, 1), round(GyrZ, 1)))
        block7 = list("Mag: %.1f,%.1f,%.1f" % (round(magX, 1), round(magY, 1), round(magZ, 1)))
        block8 = list('EOF') # Indicates end of message
        blocks = [header, block1, block2, block3, block4, block5, block6, block7, block8]
        for block in blocks:
            while len(block) < self.RADIO_PAYLOAD_SIZE:
                block.append(0)
            print(block,' ',len(block))
            self.radio.write(block)  # write the message to radio

if __name__ == '__main__':
    core = corePrimaryAircraft()


    while True:
        try:
            os.system('clear')
            core.printDataSummary()
            #core.radio.printDetails()

            core.transmitToGCS()
            time.sleep(1)

        except (KeyboardInterrupt, SystemExit):  # when you press ctrl+c
            print("\nKilling Thread...")
            core.gps.running = False
            core.gps.join()  # wait for the thread to finish what it's doing
            break

    sys.exit()