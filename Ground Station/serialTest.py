# Importing Libraries
import serial
import time

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

def readFromSerial():
    return arduino.readline()

while True:
    val = readFromSerial().decode().strip()
    if val != '':
        if val.strip() != 'EOF':
            print(val) # printing the value
        else:
            print('')