import time
import rcpy.servo as servo
import rcpy.gpio as gpio

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

# Function for calculating the speed based on the wheel diameter
# and the length of time it took to travel 4.34 centimeters which
# is 1/5 the circumference of the wheel
def calculate_speed(t_Prev, t_Now):
    distance = 4.34 # 2.17 cm is how far the wheel travels in a single state
    delta_t = t_Now - t_Prev
    return distance / delta_t

def pid_control(p, i, d, current_setting, speed):
   return current_setting

# Function for converting between an input speed from control to an output duty
# cycle for the servos
def convert_speed_to_duty(speed, max_speed):
    return speed / max_speed

def convert_duty_to_speed(motor_setting, max_speed):
    if motor_setting > 1:
        return max_speed

    return motor_setting * max_speed

# Set up GPIO pins
left_encoder_pin = 25
right_encoder_pin = 17
encoder_chip = 1
left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
right_encoder = gpio.Input(encoder_chip, right_encoder_pin)
    
servo.enable()

left_motor_setting = 1
right_motor_setting = -1

# Set up clocks to periodically update the motors
period = 0.01
leftServo.set(0.0)
rightServo.set(0.0)
clk0 = leftServo.start(period)
clk1 = rightServo.start(period)

# We wait for a short amount of time while sending 0 pulse
# before sending the real setting
time.sleep(1) 
leftServo.set(left_motor_setting)
rightServo.set(right_motor_setting)

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

# Characterizations
# Right max Speed: 22.727054005733176 cm / s
# Left max Speed: 23.912102546048775 cm / s
# Wheel Circumference: 21.7 cm

while True:
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
            # Calculate Speed
            speed = calculate_speed(left_tPrev, left_tNow)
            # Control
            left_motor_setting = pid_control(.9, 0, 0, left_motor_setting, speed)
            leftServo.set(left_motor_setting)
            print("Left Speed: {}".format(speed))

    if right_state != right_reading:
        right_state = right_reading

        if right_state == right_start_state:
            right_tPrev = right_tNow
            right_tNow = time.time()
            speed = calculate_speed(right_tPrev, right_tNow)
            right_motor_setting = pid_control(1, 0, 0, right_motor_setting, speed)
            rightServo.set(right_motor_setting)
            print("Right Speed: {}".format(speed))

clk0.stop()
clk1.stop()
servo.disable()
