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

def calculate_error(measured_speed, ideal_speed):
    return ideal_speed - measured_speed

def pid_control(p, i, d, ideal_speed, speed, motor_setting, max_speed):
   error = calculate_error(speed, ideal_speed)
   proportional_component = convert_speed_to_duty(p * error, max_speed)

   output_motor_setting = proportional_component + motor_setting
   # print("Output Motor Setting: {}, Proportional Component: {}, Input Motor Setting: {}".format(output_motor_setting, proportional_component, motor_setting))
   return output_motor_setting

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
# Wheel Circumference: 21.7

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
