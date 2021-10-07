#!/usr/bin/python3
 
import time
import serial
 
ser_handle = serial.Serial(
	port = '/dev/ttyAMA0',
	baudrate = 9600,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1
)
 
print("Raspberry's sending : ")
 
try:
    while True:
        # Send 'hehe' bytes to serial 
    	ser_handle.write(b'hehe')
    	ser_handle.flush()
    	print("hehe")
    	time.sleep(1)
except KeyboardInterrupt:
	ser.close()
