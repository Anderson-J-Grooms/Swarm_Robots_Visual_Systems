import time
import rcpy.servo as servo
import rcpy.gpio as gpio

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

import smbus
from colorCheck import *

################################################
#             CONTROL FUNCTIONS
################################################

# A function for normalizing the motor setting to the range [-1 1].
# The servo motors have a safe range of operation between those
# two points 
def norm(unbounded_setting):
    if abs(unbounded_setting) <= 1:
        return unbounded_setting
    else:
        return unbounded_setting / abs(unbounded_setting)

# Function for calculating the speed based on the wheel diameter
# and the length of time it took to travel 4.34 centimeters which
# is 1/5 the circumference of the wheel
def calculate_speed(t_Prev, t_Now):
    distance = 4.34 # 2.17 cm is how far the wheel travels in a single state
    delta_t = t_Now - t_Prev
    return distance / delta_t

# Simple funtion to calculate the error from the measured speed
def calculate_error(measured_speed, ideal_speed):
    return ideal_speed - measured_speed

# Function that implements pid control. Currently P works sufficiently and the
# other two components have not been implemented.
def pid_control(p, i, d, error, motor_setting, max_speed):
   # Since the error has units of cm/s we need to convert to us for sending to the servos
   proportional_component = convert_speed_to_duty(p * error, max_speed)

   # We need to normalize the output to a range of [-1 1]
   output_motor_setting = norm(proportional_component + motor_setting)
   print("Output Motor Setting: {}, Proportional Component: {}, Input Motor Setting: {}".format(output_motor_setting, proportional_component, motor_setting))
   return output_motor_setting

# Function for converting between an input speed from control to an output duty
# cycle for the servos
def convert_speed_to_duty(speed, max_speed):
    return norm(speed / max_speed)

# Function for converting between us for the duty cyle and cm/s for the speed of the motors
def convert_duty_to_speed(motor_setting, max_speed):
    return norm(motor_setting) * max_speed

################################################
#          END FUNCTION DECLARATIONS
################################################

################################################
#              RGB SENSOR SETUP
################################################

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

# Function for calibrating the RGB sensor for the ambient light conditions
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
            
# Function for calling the RGB sensor code and getting a color
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

calibrate_light()

################################################
#            END RGB SENSOR SETUP
################################################

# Set up GPIO pins and configure Wheel Encoders
left_encoder_pin = 25
right_encoder_pin = 17
encoder_chip = 1
left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
right_encoder = gpio.Input(encoder_chip, right_encoder_pin)

################################################
#                  SERVO SETUP
################################################

# Enable servo and calibrate light sensor
servo.enable()

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

# We wait for a short amount of time while sending 0.0 pulse
# before sending the real setting
time.sleep(1)
leftServo.set(left_motor_setting)
rightServo.set(right_motor_setting)

# Function that takes a new speed for the two motor
# and updates the necessary variables
# it then calls the servo.set() function to update
# the pulse being sent to the servos
def update_motors(left_new_speed, right_new_speed):
    global left_motor_setting, right_motor_setting
    global left_motor_speed, right_motor_speed

    left_motor_speed = left_new_speed
    right_motor_speed = right_new_speed
    left_motor_setting = convert_speed_to_duty(left_motor_speed)
    right_motor_setting = convert_speed_to_duty(right_motor_speed)
    leftServo.set(left_motor_setting)
    rightServo.set(right_motor_setting)

################################################
#              END SERVO SETUP
################################################   

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
control_current_state = "black"

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

# Main control loop
while True:
    # Take a color reading and decide if we are changing states
    state_color = get_color()
    if state_color == "white" and state_color != control_current_state:
        control_current_state = state_color
        print("STOPPING")
        update_motors(0, 0)

    elif state_color == "black" and state_color != control_current_state:
        control_current_state = state_color
        print("STARTING")
        update_motors(20, 20)

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
            left_motor_setting = pid_control(1, 0, 0, error, left_motor_setting, 23.912102546048775)
            leftServo.set(left_motor_setting)

    if right_state != right_reading:
        right_state = right_reading

        if right_state == right_start_state:
            right_tPrev = right_tNow
            right_tNow = time.time()
            speed = calculate_speed(right_tPrev, right_tNow)
            error = calculate_error(speed, right_motor_speed)
            print("Right Speed: {}, Right Setting: {} Right Error: {}".format(speed, right_motor_setting, error))
            right_motor_setting = pid_control(1, 0, 0, error, right_motor_setting, -22.727054005733176)
            rightServo.set(right_motor_setting)


clk0.stop()
clk1.stop()
servo.disable()
