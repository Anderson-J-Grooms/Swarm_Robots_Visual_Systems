import serial
import time

while (1):
	port = serial.Serial("/dev/ttyAMA0", baudrate = 115200)

	port.write(bytes("test data","utf-8"))

	time.sleep(1)
