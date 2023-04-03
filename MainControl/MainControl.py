import time
import rcpy.servo as servo
import rcpy.gpio as gpio

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

import smbus
from colorCheck import *

# Function for calculating the speed based on the wheel diameter
# and the length of time it took to travel 4.34 centimeters which
# is 1/5 the circumference of the wheel
def calculate_speed(t_Prev, t_Now):
    distance = 4.34 # 2.17 cm is how far the wheel travels in a single state
    delta_t = t_Now - t_Prev
    return distance / delta_t

def calculate_error(measured_speed, ideal_speed):
    return ideal_speed - measured_speed

def pid_control(p, i, d, ideal_speed, speed, motor_setting, max_speed):
   error = calculate_error(speed, ideal_speed)
   proportional_component = convert_speed_to_duty(p * error, max_speed)

   output_motor_setting = proportional_component + motor_setting
   print("Output Motor Setting: {}, Proportional Component: {}, Input Motor Setting: {}".format(output_motor_setting, proportional_component, motor_setting))
   return output_motor_setting

# Function for converting between an input speed from control to an output duty
# cycle for the servos
def convert_speed_to_duty(speed, max_speed):
    if abs(speed) > abs(max_speed):
        return abs(speed) / speed 
    
    return speed / max_speed

def convert_duty_to_speed(motor_setting, max_speed):
    if motor_setting > 1:
        return max_speed

    return motor_setting * max_speed

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
first = True
firstCount = 0
second = True
secondCount = 0

def calibrate_light():
    global first, firstCount, second, secondCount
    global greenBlack, greenWhite, redBlack, redWhite, blueBlack, blueWhite
    global rScale, gScale, bScale
    
    while first or second:
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
            

def get_color():
    global greenBlack, greenWhite, redBlack, redWhite, blueBlack, blueWhite
    global rScale, gScale, bScale
    # get sensor data
    data = bus.read_i2c_block_data(0x44, 0x09, 6)
    g = data[1] * 256 + data[0]
    r = data[3] * 256 + data[2]
    b = data[5] * 256 + data[4]

    try:
        r = 1.0 * (r-redBlack) / rScale
        g = 1.0 * (g-greenBlack) / gScale
        b = 1.0 * (b-blueBlack) / bScale
    except:
        print("division by zero")
    cmax = max(r, g, b)
    cmin = min(r, g, b)
    diff = cmax-cmin
    h = -1
    s = -1
    if cmax == cmin:
        h = 0
    elif cmax == r:
        h = (60.0 * ((g-b) / diff) + 360) % 360
    elif cmax == g:
        h = (60.0 * ((g-b) / diff) + 360) % 360
    elif cmax == b:
        h = (60.0 * ((g-b) / diff) + 360) % 360
    if cmax == 0:
        s = 0
    else:
        s = (diff / cmax) * 100
    v = cmax * 100
    return colorCheck(h, s, v)

# Set up GPIO pins
left_encoder_pin = 25
right_encoder_pin = 17
encoder_chip = 1
left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
right_encoder = gpio.Input(encoder_chip, right_encoder_pin)

# Enable servo and calibrate light sensor
servo.enable()
calibrate_light()

# Motor speeds
left_motor_speed = 20 # cm/s
left_motor_setting = convert_speed_to_duty(left_motor_speed, 23.912102546048775)
right_motor_speed = 20 # cm/s
right_motor_setting = convert_speed_to_duty(right_motor_speed, -22.727054005733176)

# Set up clocks to periodically update the motors
period = 0.01
leftServo.set(0.0)
rightServo.set(0.0)
clk0 = leftServo.start(period)
clk1 = rightServo.start(period)
time.sleep(1)
leftServo.set(left_motor_setting)
rightServo.set(right_motor_setting)

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

# Error variables
left_measured_error = 0
left_prev_error = 0
left_integral_error = 0
right_measure_error = 0
right_prev_error = 0
right_integral_error = 0

# Characterizations
# Right max Speed: 22.727054005733176 cm / s
# Left max Speed: 23.912102546048775 cm / s
# Wheel Circumference: 21.7 cm

while True:
    color = get_color()
    if color == "white":
        print("STOPPING")
        left_motor_speed = 0
        right_motor_speed = 0

    # Get Readings from encoders for comparison
    left_reading = left_encoder.get()
    right_reading = right_encoder.get()

    # Check if the wheel has spun
    if left_state != left_reading:
        left_state = left_reading
        # If the wheel has spun two states then we know the distance
        if left_state == left_start_state:
            # Update time
            left_tPrev = left_tNow
            left_tNow = time.time()
            # Calculate Speed and error
            speed = calculate_speed(left_tPrev, left_tNow)
            error = calculate_error(speed, left_motor_speed)
            print("Left Speed: {}, Left Setting: {}, Left Error: {}".format(speed, left_motor_setting, error))
            # Control
            left_motor_setting = pid_control(1, 0, 0, left_motor_speed, speed, left_motor_setting, 23.912102546048775)
            leftServo.set(left_motor_setting)

    if right_state != right_reading:
        right_state = right_reading

        if right_state == right_start_state:
            right_tPrev = right_tNow
            right_tNow = time.time()
            speed = calculate_speed(right_tPrev, right_tNow)
            error = calculate_error(speed, right_motor_speed)
            print("Right Speed: {}, Right Setting: {} Right Error: {}".format(speed, right_motor_setting, error))
            right_motor_setting = pid_control(1, 0, 0, right_motor_speed, speed, right_motor_setting, -22.727054005733176)
            rightServo.set(right_motor_setting)


clk0.stop()
clk1.stop()
servo.disable()
