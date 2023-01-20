# -*- coding: utf-8 -*-
"""
Main Code for the Avion Cargo Primary Aircraft Localisation System and Data Acquisition System.

"""
# TODO: - Kalman filtering, 3dim (x-y-z) to track movement (& tilt?) and positioning more accurately
#       - Increase sensor reading frequency to 5 Htz.
#       - Shield radio units
#       - Work on communication reliability and robustness
#       - Dedicated PSU for radio units
#       - Target positioning and computer vision


# General modules
import sys, os, smbus, time
from misc import detect_model
import spidev # import SPI
from math import pi, atan2

# Board specific modules
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

# Sensor Modules
from BMP388 import BMP388 # Import Pressure sensor
from LSM6DSL import LSM6DSL # Import Accelerometer and Gyro Module
from LIS3MDL import LIS3MDL # Import Compass module
from lib_nrf24 import NRF24 # Import Radio module
from GPSPoller import GpsPoller



class corePrimaryAircraft():

    # Radio configuration
    RADIO_PAYLOAD_SIZE = 32
    RADIO_DATA_RATES = [NRF24.BR_250KBPS, NRF24.BR_1MBPS, NRF24.BR_2MBPS]
    RADIO_PA_LEVELS = [NRF24.PA_MIN, NRF24.PA_LOW, NRF24.PA_HIGH, NRF24.PA_MAX]
    # Set the pipe addresses. These addresses should be entered on the receiver too.
    RADIO_WRITING_PIPE = [0xE0, 0xE0, 0xE0, 0xE0, 0xE0]
    RADIO_READING_PIPE = [0xF0, 0xF0, 0xF0, 0xF0, 0xF0]
    RADIO_AKPL_BUF = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8]

    def __init__(self):
        # Define sensor objects
        self.altimeter = BMP388(smbus.SMBus(i2c_bus))
        self.imu = LSM6DSL(smbus.SMBus(i2c_bus))
        self.compass = LIS3MDL(smbus.SMBus(i2c_bus))
        self.radio = NRF24(GPIO, spidev.SpiDev())
        self.gps = GpsPoller()

        # Initialise sensor objects
        self._initRadio()
        self._initAltimeter()
        self._initCompass()
        self._initIMU()
        self._initGPS()

        print('Initialisation Complete! \n')

        self.STATUS = 'STANDBY'
        self.altMovAverage = []
        self.failedRecv = 0
        self.okRecv = 0

        self.ref_origin = []


    def _initAltimeter(self):
        """
        Altimeter works by comparing altitude pressure to ground pressure.
        Ground pressure is used to calibrate the altimeter.
        """
        print('Setting up altimeter (BMP388) ...')
        self.calibrateAltimeter()


    def _initIMU(self):
        print('Setting up IMU (LSM6DSL) ...')

    def _initCompass(self):
        print('Setting up compass (LIS3MDL) ...')
        # Compensation and calibration?

    def _initRadio(self):
        """
        Initialize nRF24L01+ radio transceiver and communication channels.
        Settings here need to be compatible with GCS settings:
            - Payload size: 32 bytes
            - Radio Channel: #36
            - Data Rate: 125 kB
            - RADIO_WRITING_PIPE: [0xE0, 0xE0, 0xE0, 0xE0, 0xE0] (reading pipe at GCS)
            - RADIO_READING_PIPE: [0xF0, 0xF0, 0xF0, 0xF0, 0xF0] (writing pipe at GCS)


        """
        print('Setting up radio (nRF24L01+) ...')
        self.radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins
        self.radio.begin(0, spiPin)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25

        self.radio.setPayloadSize(self.RADIO_PAYLOAD_SIZE)  # set the payload size as 32 bytes
        self.radio.setChannel(0x36)  # set the channel as 76 hex
        self.radio.setRetries(5, 10)
        self.radio.setDataRate(self.RADIO_DATA_RATES[0])  # set radio data rate to 2MBPS
        self.radio.setPALevel(self.RADIO_PA_LEVELS[0])  # set PA level to LOW

        self.radio.setAutoAck(True)  # set acknowledgement as true
        self.radio.enableDynamicPayloads()
        self.radio.enableAckPayload()

        self.radio.openWritingPipe(self.RADIO_WRITING_PIPE)  # open the defined pipe for writing
        self.radio.openReadingPipe(1, self.RADIO_READING_PIPE)  # open the defined pipe for reading
        self.radio.stopListening()

    def _initGPS(self):
        # Initialize GPS Reader Thread
        print('Setting up GPS thread ...')
        self.gps.start()

    def kill(self):
        self.gps.running = False
        self.gps.join()  # wait for the thread to finish what it's doing
        sys.exit()
    def printDataSummary(self):
        """
        Print sensor data summary to terminal.
        """
        print('===============================================\n$ ')
        # Print Altimeter Data
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        print('$ ALTIMETER DATA:')
        print('$  -> Temperature = %.1f C\n$  -> Pressure = %.2f Pa\n$  -> Altitude =%.2f m\n$ ' % (
            temperature, pressure, altitude))
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

    def fetchData(self):
        # Fetch Altimeter Data
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        if len(self.altMovAverage) < 15:
            self.altMovAverage.append(altitude)
        else:
            for i in range(len(self.altMovAverage), 1):
                self.altMovAverage[i] = self.altMovAverage[i-1]
            self.altMovAverage[0] = altitude

        altitude = sum(self.altMovAverage)/len(self.altMovAverage)
        #print(self.altMovAverage)

        # Fetch Compass Data
        magX = self.compass.readMAGxCorr()
        magY = self.compass.readMAGyCorr()
        magZ = self.compass.readMAGzCorr()
        # Calculate Heading
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360

        # Fetch IMU Data
        AccX = self.imu.readACCx()
        AccY = self.imu.readACCy()
        AccZ = self.imu.readACCz()
        GyrX = self.imu.readGYRx()
        GyrY = self.imu.readGYRy()
        GyrZ = self.imu.readGYRz()
        # Fetch GPS Data
        lat, long, altGPS = self.gps.getPosition()

        data = {}
        data['temperature'] = temperature
        data['pressure'] = pressure
        data['altitude'] = altitude
        data['mag'] = [magX, magY, magZ]
        data['heading'] = heading
        data['acc'] = [AccX, AccY, AccZ]
        data['gyr'] = [GyrX, GyrY, GyrZ]
        data['GPS'] = [lat, long, altGPS]

        self.data = data

        return data

    def setupDataForTransmission(self, data):
        '''
        Organize sensor data in blocks ready for transmission.

        :input:
            - data: dictionary of the available data

        :return:
            - blocks: list of blocks to be transmitted
        '''
        temperature = data['temperature']
        pressure = data['pressure']
        altitude = data['altitude']
        magX, magY, magZ = data['mag']
        heading = data['heading']
        AccX, AccY, AccZ = data['acc']
        GyrX, GyrY, GyrZ = data['gyr']
        lat, long, altGPS = data['GPS']

        numBlocks = 9
        # header = list('$b' + str(int(numBlocks)) + ',tph' + ',lat' + ',long')
        header = list('BOF')  # Indicates beginning of message
        block1 = list("temperature: %.1f" % round(temperature, 1))
        block2 = list("pressure: %.1f" % round(pressure, 1))
        block3 = list("altitude: %.1f" % round(altitude, 1))
        block4 = list("pos:" + str(lat) + ',' + str(long))
        block5 = list("altGPS:" + str(altGPS))
        block6 = list("Acc: %.1f,%.1f,%.1f" % (round(AccX, 2), round(AccY, 2), round(AccZ, 2)))
        block7 = list("Gyr: %.1f,%.1f,%.1f" % (round(GyrX, 2), round(GyrY, 2), round(GyrZ, 2)))
        block8 = list("Mag: %.3f,%.3f,%.3f" % (round(magX, 3), round(magY, 3), round(magZ, 3)))
        # ADD COMMAND REPLIES AND REMOVE UNNECESSARY DATA FRAMES
        eof = list('EOF')  # Indicates end of message

        blocks = [header, block1, block2, block3, block4, block5, block6, block7, block8, eof]

        for block in blocks:
            while len(block) < self.RADIO_PAYLOAD_SIZE: # Fill remaining bytes with zeros
                block.append(0)


        return blocks

    def transmitToGCS(self, blocks):
        '''
        Transmits sensor data to ground station.

        The nRF24L01+ module can only send a maximum of 32 bytes at a time (i.e one block).
        Sensor information must be split in blocks of 32 bytes.
        Header block sends information concerning message structure and contents.
        One transmission is concluded by an 'EOF' message.

        :return: None
        '''


        print('Transmitting to ground station...\n')

        # Confirm that the radio is in transmit mode
        self.radio.stopListening()

        # Begin transmitting the message
        for block in blocks:
            self.radio.write(block)  # write the message to radio

        # Confirm that the radio is in listening mode
        self.radio.startListening()

    def receiveFromGCS(self, timeout=1.0):
        """
        Listen to any transmission coming fromm the ground station.
        Wait one second before timeout.
        """
        print('\nListening to ground station...')
        recv_blocks = []
        recv_buffer = []

        t1 = time.time()
        while len(recv_blocks) == 0:
            # Check for timeout...
            if (time.time() - t1) > timeout:
                print('Heard nothing from ground station...')

                return None  # Leave function if nothing received

            while self.radio.available([1]):
                # If transmission received, receive and process the message
                self.radio.read(recv_buffer, self.radio.getDynamicPayloadSize())

                # Convert transmission to a readable format
                for i, val in enumerate(recv_buffer):
                    # convert integers from transmission to its unicode character
                    if (val >= 32 and val <= 126):
                        recv_buffer[i] = chr(val)

                recv_blocks.append(recv_buffer)

        print('Received from GCS: ')
        for buf in recv_blocks:
            print(buf)
        print('Stopped listening to ground station...')

        return recv_blocks

    def processRecv(self, recv_blocks):
        '''
        Process the received buffer in order to execute a command.
        '''
        if recv_blocks is None:
            return self.STATUS

        for recv_buffer in recv_blocks:
            try:
                recv_comm = ''.join(str(e) for e in recv_buffer)
                if recv_comm.find("$RESET") != -1:
                    # To Define
                    # Put core in 'STANDBY' mode
                    pass

                elif recv_comm.find("$ARM") != -1:
                    # If PA is in STANDBY Mode, put it in READY mode, which begins the mission
                    # Disables calibration, to enable calibration, send a $STANDBY command

                    if self.STATUS != "ARMED":
                        self.STATUS = 'ARMED'

                        # Indicate to GCS that message has been received and that the
                        # PA computer is ready and ARMED

                        header = list('BOF')  # Indicates beginning of message
                        block = list('@ARMED')
                        eof = list('EOF')  # Indicates end of message

                        blocks = [header, block, eof]

                        for block in blocks:
                            while len(block) < self.RADIO_PAYLOAD_SIZE:  # Fill remaining bytes with zeros
                                block.append(0)

                        self.transmitToGCS(blocks)  # write the message to radio

                        # ARMING represents mission begin; set origins
                        # Expect a few seconds delay here

                        self.calibrateAltimeter()

                        self.setOrigin()

                        # Maybe use isAckPayloadAvailable() to confirm message reception
                elif recv_comm.find("$KILL") != -1:
                    self.kill()

                elif recv_comm.find("$STANDBY") != -1:
                    if self.STATUS != "STANDBY":
                        self.STATUS = 'STANDBY'

                        # Indicate to GCS that message has been received and that the
                        # PA computer STATUS is set to STANDBY
                        header = list('BOF')  # Indicates beginning of message
                        block = list('@STANDBY')
                        eof = list('EOF')  # Indicates end of message

                        blocks = [header, block, eof]

                        for block in blocks:
                            # Fill remaining bytes with zeros
                            while len(block) < self.RADIO_PAYLOAD_SIZE:
                                block.append(0)

                        self.transmitToGCS(blocks)  # write the message to radio

                elif recv_comm.find("$RELEASE") != -1:
                    pass

                elif recv_comm.find("$CAL_ALTIMETER") != -1:
                    if self.STATUS == 'STANDBY':
                        self.calibrateAltimeter()

                elif recv_comm.find("$SET_ORIGIN") != -1:
                    if self.STATUS == 'STANDBY':
                        self.setOrigin()

                else:
                    pass
            except Exception as e:
                print(e)


        return self.STATUS

    def receiveFromGCSOld(self):
        """
        Listen to any transmission coming fromm the ground station.
        Wait one second before timeout.
        """
        print('\nListening to ground station...')
        t1 = time.time()
        # Wait for a transmission until timeout
        while not self.radio.available([1]):
            # Check for timeout...
            if (time.time() - t1) > 1.0:
                print('Heard nothing from ground station...')
                return None  # Leave function if nothing received

            time.sleep(1 / 100)

        # If transmission received, receive and process the message
        recv_buffer = []
        self.radio.read(recv_buffer, self.radio.getDynamicPayloadSize())

        # Convert transmission to a readable format
        for i, val in enumerate(recv_buffer):
            # convert integers from transmission to its unicode character
            if (val >= 32 and val <= 126):
                recv_buffer[i] = chr(val)

        print('Received from GCS: ')
        print(recv_buffer)
        print('Stopped listening to ground station...')

        return recv_buffer

    def calibrateAltimeter(self, num=100):
        '''
        Calibrate the altimeter so that the current pressure readings is equal to 0m of altitude.
        Grabs 100 measurements. Cannot be used if vehicle status is "ARMED".

        '''

        vals = []
        for i in range(num):
            pressure = self.altimeter.get_temperature_and_pressure_and_altitude()[1]
            vals.append(pressure)
            time.sleep(0.010)

        av = sum(vals)/len(vals)

        self.altimeter.setGroundPressure(av)
        print(' -> Ground Pressure Level = %.2f Pa' % (av))

    def setOrigin(self, num=10):
        '''
        Set the origin coordinates of the local navigational reference frame (LNRF).
        It is used to calculate the position of the Primary Aircraft in the LNRF.
        '''

        lats = []
        longs = []
        alts = []

        for i in range(num):
            lat, long, altGPS = self.gps.getPosition()
            lats.append(lat)
            longs.append(long)
            alts.append(altGPS)
            time.sleep(0.3)

        av = [sum(lats) / len(lats),
              sum(longs) / len(longs),
              sum(alts) / len(alts)]


        self.ref_origin = av
        print(' -> Origin Coordinates: %.5f°N, %.5f°E' % (av[0], av[1]))


    def waitForMissionBegin(self, timeout):
        """
        Wait for mission begin. Mission begins when the radio reads a '$ARM' command.
        Once this command is received, the radio sends a '@ARMED' confirmation message.
        """

        # Confirm radio is in listening mode
        self.radio.startListening()

        # Wait for a command from the ground station
        stat = self.STATUS

        # Wait for status to change
        while stat.upper() == 'STANDBY':
            # Clear terminal on each iteration
            os.system('clear')

            # Start timer
            t1 = time.time()

            # Indicate to GCS that PA computer is ready and waiting for commands
            header = list('BOF')  # Indicates beginning of message
            block = list('@STANDBY')
            eof = list('EOF')  # Indicates end of message

            blocks = [header, block, eof]

            for block in blocks:
                while len(block) < self.RADIO_PAYLOAD_SIZE:  # Fill remaining bytes with zeros
                    block.append(0)

            self.transmitToGCS(blocks)  # Write the message to radio

            recv_blocks = self.receiveFromGCS(timeout)  # Wait for a message from the GCS
            stat = self.processRecv(recv_blocks)  # Process any received messages

            # Wait a loop timeout before the next transmission
            dt = time.time() - t1
            print('Loop dt: ' + str(round(dt, 3)) + ' s')
            if (timeout - dt) > 0:
                time.sleep(timeout - dt)

        return stat

    def analyse_frame(self):
        pass
    def missionLoop(self, timeout):
        okRecv = 0
        failedRecv = 0
        stat = self.STATUS
        detect_target = False

        while stat.upper() == 'ARMED':

            # Clear terminal on each iteration
            os.system('clear')
            t1 = time.time()
            # core.printDataSummary()
            # core.radio.printDetails()

            # Fetch sensor data
            while detect_target:
                data = core.fetchData()
                detect_target = self.analyse_frame()
                data['STATUS'] = '@ARMED'
                # Ready Data for transmission
                blocks = core.setupDataForTransmission(data)

            # Transmit sensor data to GCS
            core.transmitToGCS(blocks)
            # Receive any transmissions from the GCS
            recv_blocks = core.receiveFromGCS(timeout)

            if recv_blocks == None:
                failedRecv += 1
            else:
                okRecv += 1

            print('Success Rate: ' + str(round((okRecv) / (okRecv + failedRecv) * 100, 1)) + '%\n')
            # Process the received buffer from the GCS
            stat = core.processRecv(recv_blocks)
            # Wait a loop timeout before the next transmission
            dt = time.time() - t1
            print('@Loop dt: ' + str(round(dt, 3)) + ' s\n')
            print('@GROUND PRESSURE: ' + str(self.altimeter.groundPressure))
            print('@REF_ORIGIN: %.5f°N, %.5f°E' % (self.ref_origin[0], self.ref_origin[1]))
            if not (timeout - dt) <= 0:
                time.sleep(timeout - dt)

        return stat
    def main(self):

        iter_rate = 5
        timeout = 1 / iter_rate
        stat = self.STATUS

        # Core loop, break on keyboard interrupt (Ctr + C)
        while True:
            try:
                if stat.upper() == 'ARMED':
                    # Enter mission loop;
                    # Begins sending sensor data to GCS
                    # Engage Navigation and Computer Vision
                    stat = self.missionLoop(timeout)

                elif stat.upper() == 'STANDBY':
                    # Wait for '$ARM' command from GCS
                    stat = self.waitForMissionBegin(timeout)

                else:
                    pass

            except (KeyboardInterrupt, SystemExit):  # When you press ctrl+c
                print("\nKilling Core...")
                return False






if __name__ == '__main__':
    # PA Avionics core (main) definition class. Handles sensors, localisation and targeting (computer vision).
    core = corePrimaryAircraft()
    time.sleep(1.0)
    core.main()
    sys.exit()