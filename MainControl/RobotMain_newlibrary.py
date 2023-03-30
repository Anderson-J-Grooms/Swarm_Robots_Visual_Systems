import time
import rcpy.servo as servo
import rcpy.gpio as gpio

from rcpy.servo import servo8 as leftServo
from rcpy.servo import servo1 as rightServo

#Set up GPIO pins
left_encoder_pin = 25
right_encoder_pin = 17
encoder_chip = 1
left_encoder = gpio.Input(encoder_chip, left_encoder_pin)
right_encoder = gpio.Input(encoder_chip, right_encoder_pin)
    
servo.enable()

left_motor_setting = 1
right_motor_setting = -1
period = 0.01

leftServo.set(0.0)
rightServo.set(0.0)
clk0 = leftServo.start(period)
clk1 = rightServo.start(period)

# We wait for a short amount of time while sending 0 pulse
# before sending the real setting
time.sleep(2) 
leftServo.set(left_motor_setting)
rightServo.set(right_motor_setting)

timer = 10000

while timer > 0:
    left_state = left_encoder.get()
    right_state = right_encoder.get()
    print("Left State: %s, Right State: %s" % (left_state, right_state))
    timer = timer - 1;
    

clk0.stop()
clk1.stop()
servo.disable()
