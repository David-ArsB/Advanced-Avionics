"""
Main Code for the Avion Cargo Primary Aircraft Localisation System.

"""

import sys, os, smbus, time
from misc import detect_model

if detect_model() == 'Hardkernel ODROID-C4\x00':
    i2c_bus = 0x00
    import Odroid.GPIO as GPIO
    GPIO.setmode(GPIO.WIRINGPI)  # set the gpio mode
    spiPin = 6

elif detect_model() == 'Raspberry Pi 3 Model B Rev 1.2\x00':
    i2c_bus = 0x01
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)  # set the gpio mode
    spiPin = 25

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
    RADIO_AKPL_BUF = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8]

    def __init__(self):
        self.altimeter = BMP388(smbus.SMBus(i2c_bus))
        self.imu = LSM6DSL(smbus.SMBus(i2c_bus))
        self.compass = LIS3MDL(smbus.SMBus(i2c_bus))
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
        self.radio.begin(0, spiPin)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25

        self.radio.setPayloadSize(self.RADIO_PAYLOAD_SIZE)  # set the payload size as 32 bytes
        self.radio.setChannel(0x36)  # set the channel as 76 hex
        self.radio.setDataRate(self.RADIO_DATA_RATES[0])  # set radio data rate to 2MBPS
        self.radio.setPALevel(self.RADIO_PA_LEVELS[0])  # set PA level to LOW

        self.radio.setAutoAck(True)  # set acknowledgement as true
        self.radio.enableDynamicPayloads()
        self.radio.enableAckPayload()

        self.radio.openWritingPipe(self.RADIO_WRITING_PIPE)  # open the defined pipe for writing
        self.radio.openReadingPipe(0, self.RADIO_READING_PIPE)  # open the defined pipe for reading
        self.radio.stopListening()

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
        lat, long, altGPS = self.gps.getPosition()
        print('$ GPS DATA:')
        print('$  -> Latitude = %.8f N, Longitude = %.8f E, Altitude = %.1f\n$ ' % (lat, long, altGPS))
        print('===============================================\n')

    def transmitToGCS(self):
        '''
        Transmits sensor data to ground station.

        The nRF24L01+ module can only send a maximum of 32 bytes at a time (i.e one block).
        Sensor information must be split in blocks of 32 bytes.
        Header block sends information concerning message structure and contents.

        :return: None
        '''
        print('Transmitting to ground station...\n')

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
        lat, long, altGPS = self.gps.getPosition()

        numBlocks = 9
        header = list('$b'+ str(int(numBlocks)) + ',tph' + ',lat' + ',long')
        header = list('BOF') # Indicates beginning of message
        block1 = list("temperature: %.1f" % round(temperature/100, 1))
        block2 = list("pressure: %.1f" % round(pressure/100, 1))
        block3 = list("altitude: %.1f" % round(altitude/100, 1))
        block4 = list("pos:" + str(lat) + ',' + str(long))
        block5 = list("altGPS:" + str(altGPS))
        block6 = list("Acc: %.1f,%.1f,%.1f" % (round(AccX, 2), round(AccY, 2), round(AccZ, 2)))
        block7 = list("Gyr: %.1f,%.1f,%.1f" % (round(GyrX, 2), round(GyrY, 2), round(GyrZ, 2)))
        block8 = list("Mag: %.3f,%.3f,%.3f" % (round(magX, 3), round(magY, 3), round(magZ, 3)))
        block9 = list('EOF') # Indicates end of message

        blocks = [header, block1, block2, block3, block4, block5, block6, block7, block8, block9]

        for block in blocks:
            while len(block) < self.RADIO_PAYLOAD_SIZE:
                block.append(0)
            print(block,' - ',len(block))
            self.radio.write(block)  # write the message to radio
            time.sleep(10 / 100)
            # if self.radio.isAckPayloadAvailable():
            #     pl_buffer = []
            #     self.radio.read(pl_buffer, self.radio.getDynamicPayloadSize())
            #     print("Received back:"),
            #     print(pl_buffer)
            # else:
            #     print("Received: Ack only, no payload")

    def receiveFromGCS(self):
        self.radio.startListening()
        t1 = time.time()
        while not self.radio.available(self.RADIO_READING_PIPE):
            if (time.time() - t1) > 1:
                return None
            time.sleep(1 / 100)

        recv_buffer = []
        self.radio.read(recv_buffer, self.radio.getDynamicPayloadSize())
        #print("Received:")
        #print(recv_buffer)

        #self.radio.writeAckPayload(1, self.RADIO_AKPL_BUF, len(self.RADIO_AKPL_BUF))
        #print("Loaded payload reply:")
        #print(self.RADIO_AKPL_BUF)


        self.radio.stopListening()
        print('Received from GCS: ')
        print(recv_buffer)
        return recv_buffer

if __name__ == '__main__':
    core = corePrimaryAircraft()



    while True:
        try:
            os.system('clear')
            #core.printDataSummary()
            #core.radio.printDetails()

            core.transmitToGCS()
            core.receiveFromGCS()
            time.sleep(1.0)


        except (KeyboardInterrupt, SystemExit):  # when you press ctrl+c
            print("\nKilling Thread...")
            core.gps.running = False
            core.gps.join()  # wait for the thread to finish what it's doing
            break

    sys.exit()