import sys
from pathlib import Path
cwd = Path.cwd()
mod_path = Path(__file__).parent.parent
print(mod_path)
sys.path.insert(0, str(mod_path)+r'/Ground Station/lib_nrf24-master/lib_nrf24-master')

from misc import detect_model

if detect_model() == 'Hardkernel ODROID-C4\x00':
    import Odroid.GPIO as GPIO
elif detect_model() == 'Raspberry Pi 3 Model B Rev 1.2\x00':
    import RPi.GPIO as GPIO

import time  # import time library

import spidev

from lib_nrf24 import NRF24  # import NRF24 library

print('Test 1')
GPIO.setmode(GPIO.BCM)  # set the gpio mode
print('Test 2')
# set the pipe address. this address shoeld be entered on the receiver alo

pipes = [[0xE0, 0xE0, 0xE0, 0xE0, 0xE0], [0xF0, 0xF0, 0xF0, 0xF0, 0xF0]]

radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins
print('Test 2')
radio.begin(0, 25)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25

radio.setPayloadSize(32)  # set the payload size as 32 bytes

radio.setChannel(0x76)  # set the channel as 76 hex

radio.setDataRate(NRF24.BR_2MBPS)  # set radio data rate

radio.setPALevel(NRF24.PA_MIN)  # set PA level

radio.setAutoAck(True)  # set acknowledgement as true

radio.enableDynamicPayloads()

radio.enableAckPayload()

radio.openWritingPipe(pipes[0])  # open the defined pipe for writing

radio.printDetails()  # print basic detals of radio

sendMessage = list("Hi..Arduino UNO")  # the message to be sent
print(sendMessage)
while len(sendMessage) < 32:
    sendMessage.append(0)

while True:

    start = time.time()  # start the time for checking delivery time

    radio.write(sendMessage)  # just write the message to radio

    print("Sent the message: {}".format(sendMessage))  # print a message after succesfull send

    radio.startListening()  # Start listening the radio

    while not radio.available(0):

        time.sleep(1 / 100)

        if time.time() - start > 2:
            print("Timed out.")  # print errror message if radio disconnected or not functioning anymore

            break

    radio.stopListening()  # close radio

    time.sleep(3)  # give delay of 3 seconds