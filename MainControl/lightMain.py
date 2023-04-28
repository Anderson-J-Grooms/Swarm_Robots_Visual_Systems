import time
import rcpy.servo as servo
import rcpy.gpio as gpio

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

import smbus
from colorCheck import *

# Set up GPIO pins
left_encoder_pin = 25
right_encoder_pin = 17
encoder_chip = 1
left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
right_encoder = gpio.Input(encoder_chip, right_encoder_pin)
    
servo.enable()

left_motor_setting = 1
right_motor_setting = -1
period = 0.01

# Set up clocks to periodically update the motors
leftServo.set(0.0)
rightServo.set(0.0)

# We wait for a short amount of time while sending 0 pulse
# before sending the real setting

# Time variables
left_tPrev = time.time()
left_tNow = left_tPrev
right_tPrev = time.time()
right_tNow = left_tPrev

# State variables
left_start_state = left_encoder.get()
left_state = left_start_state
right_start_state = right_encoder.get()
right_state = right_start_state

#light sensor data
# Get I2C bus
bus=smbus.SMBus(1)
bus.write_byte_data(0x44, 0x01, 0x0D)
greenBlack = 0
greenWhite = 0
redBlack = 0
redWhite = 0 
blueBlack = 0
blueWhite = 0
rScale = 1
gScale = 1
bScale = 1
r = 0
g = 0
b = 0
first = True
firstCount = 0
second = True
secondCount = 0

while True:
    # get sensor data
    data = bus.read_i2c_block_data(0x44, 0x09, 6)
    g = data[1] * 256 + data[0]
    r = data[3] * 256 + data[2]
    b = data[5] * 256 + data[4]
    
    # calibrate sensor on first 2 loops
    if first:
        greenBlack = min(g, greenBlack)
        redBlack = min(r, redBlack)
        blueBlack = min(b, blueBlack)
        firstCount = firstCount + 1
        if firstCount >= 2000:
            first = False
            print("Switch to white and press enter to continue")
            input()
    elif second:
        greenWhite = max(g, greenWhite)
        redWhite = max(r, redWhite)
        blueWhite = max(b, blueWhite)
        rScale = redWhite-redBlack
        gScale = greenWhite-greenBlack
        bScale = blueWhite-blueBlack
        secondCount = secondCount + 1
        if secondCount >= 2000:
            second = False
            print("press enter to continue")
            input()
            clk0 = leftServo.start(period)
            clk1 = rightServo.start(period)
            time.sleep(1)
    else:
        try: 
            r = 1.0* (r-redBlack) / rScale
            g = 1.0* (g-greenBlack) / gScale
            b = 1.0* (b-blueBlack) / bScale
        except:
            print("division by zero")
        cmax = max(r,g,b)
        cmin = min(r,g,b)
        diff = cmax-cmin
        h=-1
        s=-1
        if cmax == cmin:
            h=0
        elif cmax == r:
            h = (60.0 * ((g-b) / diff) + 360) % 360
        elif cmax == g:
            h = (60.0 * ((g-b) / diff) + 360) % 360
        elif cmax == b:
            h = (60.0 * ((g-b) / diff) + 360) % 360
        if cmax == 0:
            s=0
        else:
            s = (diff / cmax) * 100
        v = cmax * 100
        color = colorCheck(h,s,v)

        if color == "white":
            leftServo.set(left_motor_setting)
            rightServo.set(right_motor_setting)
        else:
            leftServo.set(0)
            rightServo.set(0)
clk0.stop()
clk1.stop()
servo.disable()
