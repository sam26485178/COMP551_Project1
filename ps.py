import sys, rone
## 2015-01-28 v1.1, fixed old bump sensor function definitions

########  part 0: helper functions ########
# global PWM setting for all motion
# oyu might need to increase this a bit if your robot doesn't move
MOTOR_PWM = 65

# stops the robot for the argument time
# arguments: time
# return: nothing
def move_stop(time):
    # student code start
    rone.motor_set_pwm('r', 0);
    rone.motor_set_pwm('l', 0);
    sys.sleep(time);
    # student code start
    

# moves forward for the argument time
# arguments: time
# return: nothing
def move_forward(time):
    # student code start
    if time < 0:
        rone.motor_set_pwm('r', 0 - MOTOR_PWM)
        rone.motor_set_pwm('l', 0 - MOTOR_PWM)
    else:
        rone.motor_set_pwm('r', MOTOR_PWM)
        rone.motor_set_pwm('l', MOTOR_PWM)
    sys.sleep(abs(time))
    # student code end

    
# rotate right for the argument time
# arguments: time
# return: nothing
def move_rotate_right(time):
    # student code start
    if time < 0:
        rone.motor_set_pwm('r', 0)
        rone.motor_set_pwm('l', 0 - MOTOR_PWM)
    else:
        rone.motor_set_pwm('r', 0)
        rone.motor_set_pwm('l', MOTOR_PWM)
    sys.sleep(abs(time))
    # student code end
    

# rotate left for the argument time
# arguments: time
# return: nothing
def move_rotate_left(time):
    # student code start
    if time < 0:
        rone.motor_set_pwm('r', 0 - MOTOR_PWM)
        rone.motor_set_pwm('l', 0)
    else:
        rone.motor_set_pwm('r', MOTOR_PWM)
        rone.motor_set_pwm('l', 0)
    sys.sleep(abs(time))
    # student code end


    

########  part 1: square motion ########

# drives the robot in a square
# arguments: nothing
# return: nothing
def square_motion():
    # wait for 2 seconds to let you unplug the robot...
    sys.sleep(2000)
    for i in range(1,5):
        # student code start
        move_forward(1500)
        move_rotate_right(700)
        # student code end
    move_stop(100)




########  part 2: move towards light ########
# compute and return the difference between the left and the right light sensor
# arguments: nothing
# return: difference between left sensor and right sensor
def light_diff():
    # student code start
    light_fl = rone.light_sensor_get_value('fl')
    light_fr = rone.light_sensor_get_value('fr')
    diff = light_fl - light_fr
    # student code end
    return diff


# Move towards Light! Use the structure below, and the movement helper functions from above
# arguments: nothing
# return: nothing
def light_follow():
    diff_start = light_diff()
    print "diff_start", diff_start
    sys.sleep(1000)
    while True:
        diff = light_diff() - diff_start
        print "diff", diff
        # student code start
        tmp = diff_start
        diff_start = light_diff()
        if (diff_start > 2):
            move_rotate_left(abs(diff+tmp)/2)
        elif (diff_start < -2):
            move_rotate_right(abs(diff+tmp)/2)
        else:
            move_forward(200)
        # student code end




########  part 3: avoid obstacles with bump sensors ########
# Checks the bump sensor for impacts from the left
# arguments: nothing
# return: True if the bump sensor is pressed from the left, False otherwise
def bump_left_get_value():
    bump_sensors = rone.bump_sensors_get()
    if (0 in bump_sensors) or (1 in bump_sensors):
        return True
    else:
        return False

# Checks the bump sensor for impacts from the frontt
# arguments: nothing
# return: True if the bump sensor is pressed from the front, False otherwise
def bump_front_get_value():
    bump_sensors = rone.bump_sensors_get()
    if (0 in bump_sensors) and (7 in bump_sensors):
        return True
    else:
        return False

# Checks the bump sensor for impacts from the right
# arguments: nothing
# return: True if the bump sensor is pressed from the right, False otherwise
def bump_right_get_value():
    bump_sensors = rone.bump_sensors_get()
    if (6 in bump_sensors) or (7 in bump_sensors):
        return True
    else:
        return False


# Move the robot away from obstacles using the bump sensors
# arguments: nothing
# return: nothing
def bump_avoid():
    print "bump_avoid()"
    sys.sleep(1000)
    while True:
        # student code start
        if (bump_left_get_value()):
            move_forward(-500)
            move_rotate_right(800)
        elif (bump_front_get_value() or bump_right_get_value()):
            move_forward(-500)
            move_rotate_left(800)
        else:
            move_forward(500)
        # student code end
            


########  part 4: avoid obstacles with IR sensors ########

# Use the IR sensors to detect obstacles
# arguments: nothing
# return: tuple of booleans (obs_front, obs_left, obs_right)
def obstacle_detect():
    obs_front = False
    obs_left = False
    obs_right = False
    rone.ir_comms_send_message();
    sys.sleep(20)
    msg = rone.ir_comms_get_message()
    if msg != None:
        (msg, recv_list, xmit_list, range_bits) = msg
        if (0 in recv_list) and (7 in recv_list):
            obs_front = True
        if (0 in recv_list) or (1 in recv_list):
            obs_left = True
        if (6 in recv_list) or (7 in recv_list):
            obs_right = True
    return (obs_front, obs_left, obs_right)



# Move away from obstacles! Use the structure below, and the movement helper functiosn from above
# arguments: nothing
# return: nothing
def obstacle_avoid():
    sys.sleep(1000)
    while True:
        (obs_front, obs_left, obs_right) = obstacle_detect()
        print obs_front, obs_left, obs_right
        # student code start
        if (obs_left):
            move_forward(-500)
            move_rotate_right(800)
        elif (obs_right or obs_front):
            move_forward(-500)
            move_rotate_left(800)
        else:
            move_forward(500)
        # student code end


def main():
    while True:
        print bump_left_get_value(), bump_front_get_value(), bump_right_get_value()

########  Initial function calls to test different parts of the code ########
#move_test()
square_motion()

#light_diff_test()
#light_follow()

#bump_test()        
#bump_avoid()

#obstacle_detect_test()
#obstacle_avoid()
