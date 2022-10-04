#!/bin/python3

import serial
import time
from publisher import x_publisher

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

for i in range(500):
    line = ser.readline()   # read a byte
    if line:
        string = line.decode()  # convert the byte string to a unicode string
        x_publisher(string, 100)

ser.close()

