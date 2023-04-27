import Adafruit_BBIO.UART as UART
import serial
from time import sleep

UART.setup("UART1")

ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200)
ser.close()
ser.open()
while ser.isOpen():
    print("Serial is open!")
    #data = ser.read()
    #sleep(.03)
    #data_left = ser.inWaiting()
    #data += ser.read(data_left)
    data = ser.readline()
    print(data.decode('utf-8'))

ser.close()

