import sys
from pathlib import Path
cwd = Path.cwd()
mod_path = Path(__file__).parent.parent
print(mod_path)
sys.path.insert(0, str(mod_path)+r'/Ground Station/lib_nrf24-master/lib_nrf24-master')
import RPi.GPIO as GPIO  # import gpio
import time  # import time library
import spidev
from lib_nrf24 import NRF24  # import NRF24 library

GPIO.setmode(GPIO.BCM)  # set the gpio mode
# set the pipe address. this address should be entered on the receiver also
pipes = [[0xE0, 0xE0, 0xF1, 0xF1, 0xE0], [0xF1, 0xF1, 0xF0, 0xF0, 0xE0]]
radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins
radio.begin(0, 25)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25
radio.setPayloadSize(32)  # set the payload size as 32 bytes
radio.setChannel(0x76)  # set the channel as 76 hex
radio.setDataRate(NRF24.BR_1MBPS)  # set radio data rate
radio.setPALevel(NRF24.PA_MIN)  # set PA level
radio.setAutoAck(True)  # set acknowledgement as true
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.openWritingPipe(pipes[0])  # open the defined pipe for writing
radio.printDetails()  # print basic details of radio
sendMessage = list("Hi..Arduino UNO")  # the message to be sent

while len(sendMessage) < 32:
    sendMessage.append(0)

while True:
    start = time.time()  # start the time for checking delivery time
    radio.write(sendMessage)  # just write the message to radio
    print("Sent the message: {}".format(sendMessage))  # print a message after successfull send
    radio.startListening()  # Start listening the radio
    while not radio.available(0):
        time.sleep(1 / 100)
        if time.time() - start > 2:
            print("Timed out.")  # print error message if radio disconnected or not functioning anymore
            break

    radio.stopListening()  # close radio
    time.sleep(3)  # give delay of 3 seconds