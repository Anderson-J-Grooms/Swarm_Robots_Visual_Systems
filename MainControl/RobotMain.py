import time
from _encodercontrol_cffi import ffi, lib


tPrev = time.time()
tNow = time.time()

leftPinE = ffi.new("int *")
leftPinE[0] = 0
rightPinE = ffi.new("int *")
leftPinE[0] = 1

leftMotorPin = ffi.new("int *")
leftMotorPin[0] = 8
rightMotorPin = ffi.new("int *")
rightMotorPin[0] = 1

leftSpeed = ffi.new("double *")
leftSpeed[0] = 1
rightSpeed = ffi.new("double *")
rightSpeed[0] = -1

left_tPrev = time.time()
left_tNow = left_tPrev
right_tPrev = time.time()
right_tNow = right_tPrev

lib.init_control()

# left_state = lib.get_state(leftPinE[0])
# right_state = lib.get_state(rightPinE[0])
# left_start_state = left_state
# right_start_state = right_state

while True:
    # lib.set_motor(leftMotorPin[0], leftSpeed[0]) 
    # lib.set_motor(rightMotorPin[0], rightSpeed[0])
    lib.rc_servo_send_pulse_normalized(leftMotorPin[0], leftSpeed[0])
    lib.rc_servo_send_pulse_normalized(rightMotorPin[0], rightSpeed[0])
   # if left_state != lib.get_state(leftPinE[0]):
   #     left_state = lib.get_state(leftPinE[0])
   #     if left_state == left_start_state:
   #         left_tPrev = left_tNow
   #         left_tNow = time.time()
            # Calculate speed
            #print("Left state: " , left_state , " Prev T: ", left_tPrev , " Curr T: " , left_tNow)

   # if right_state != lib.get_state(rightPinE[0]):
   #     right_state = lib.get_state(rightPinE[0])
   #     if right_state == right_start_state:
   #         right_tPrev = right_tNow
   #         right_tNow = time.time()
            # Calculate speed
            #print("Right state: " , right_state , " Prev T: ", right_tPrev , " Curr T: " , right_tNow)

