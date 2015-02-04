import rone, sys, math, math2, leds, velocity, poseX, motionX

################################################################################
##                      Student code - hand this section in                   ##
################################################################################

#### Pose estimator ####
WHEEL_BASE = 78
ENCODER_MM_PER_TICKS = 0.0625

#update the pose state
def pose_update(pose_state):
    # 1. Get the left and right encoder ticks
    ticks_left = rone.encoder_get_ticks('l')
    ticks_right = rone.encoder_get_ticks('r')
    # 2. Compute the left and right delta ticks
    # Don't forget to use encoder_delta_ticks() to properly compute the change in tick values
    delta_ticks_left = velocity.encoder_delta_ticks(ticks_left, pose_state['ticksL'])
    delta_ticks_right = velocity.encoder_delta_ticks(ticks_right, pose_state['ticksR'])
    # 3. compute the left and right distance for each wheel
    # cast the delta ticks from step 2 to floats before you do the distance computation
    dL = ENCODER_MM_PER_TICKS * (1.0 * delta_ticks_left)
    dR = ENCODER_MM_PER_TICKS * (1.0 * delta_ticks_right)
    # 4. save the left and right ticks to pose_state so we can measure the difference next time
    pose_state['ticksL'] = ticks_left
    pose_state['ticksR'] = ticks_right
    # 5. Compute the distance traveled by the center of the robot in millimeters
    dC = 0.5 * (dL + dR) 
    # 6. Add the distance to the odometer variable in pose_state
    pose_state['odometer'] += abs(dC)
    # 7. compute the arc angle in radians
    # don't call atan here, use the small angle approximation: arctan(theta) ~ theta
    theta = (dR - dL) / WHEEL_BASE
    # 8. finally, update x, y, and theta, and save them to the pose state
    # use math2.normalize_angle() to normalize theta before storing it in the pose_state
    pose_state['theta'] = math2.normalize_angle(pose_state['theta'] + theta)
    pose_state['x'] += dC * math.cos(pose_state['theta'])
    pose_state['y'] += dC * math.sin(pose_state['theta'])



#### Waypoint controller constants ####
MOTION_CAPTURE_DISTANCE = 16
MOTION_RELEASE_DISTANCE = 32
MOTION_CAPTURE_ANGLE = math.pi/2
MOTION_RELEASE_ANGLE = math.pi/10
MOTION_TV_MIN = 20
MOTION_TV_GAIN = 3
MOTION_RV_GAIN = 1300
MOTION_RV_MAX = 7000

    
# Convert rectangular to polar
# return a tuple of the form (r, theta) 
def topolar(x, y):
    # student code start
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    #student code end
    return (r, theta)


# compute the distance and heading to the goal position
# return a tuple of the form: (goal_distance, goal_heading, robot_heading)
def compute_goal_distance_and_heading(goal_position, robot_pose):
    # student code start
    [goal_distance, goal_heading] = topolar((goal_position[0] - robot_pose[0]), goal_position[1] - robot_pose[1])
    robot_heading = robot_pose[2]
    # student code end
    return (goal_distance, goal_heading, robot_heading)


# Compute the smallest angle difference between two angles
# This difference will lie between (-pi, pi]
def smallest_angle_diff(current_angle, goal_angle):
    # student code start
    diff = goal_angle - current_angle
    if diff > math.pi:
        diff = diff - 2*math.pi
    if diff < -math.pi:
        diff = diff + 2*math.pi
    # student code end
    return diff


# compute the tv profile for the velocity controller.
# this should match the plot from the handout
def motion_controller_tv(d, tv_max):
    # student code start
    tvtemp = MOTION_TV_GAIN * d + MOTION_TV_MIN
    tv = velocity.clamp(tvtemp, tv_max)
    # student code end
    return tv


# compute the rv controller for the velocity controller
# this should bound the value to MOTION_RV_MAX
def motion_controller_rv(heading, heading_to_goal):
    # student code start
    bearing_error = smallest_angle_diff(heading, heading_to_goal)
    rvt = MOTION_RV_GAIN * bearing_error
    rv = math2.bound(rvt, MOTION_RV_MAX)
    # student code end
    return (rv, bearing_error)




################################################################################
##                         Helper and main function                           ##
##                Distribution code - do not print or hand in                 ##
################################################################################

MOTION_TV = 100
LED_BRIGHTNESS = 40
MODE_INACTIVE = 0
MODE_ACTIVE = 1

def waypoint_motion(): 
    velocity.init(0.22, 40, 0.5, 0.1)
    leds.init()
    poseX.init(pose_update)
    motionX.init(compute_goal_distance_and_heading, motion_controller_tv, motion_controller_rv)

    pose_estimator_print_time = sys.time()
    mode = MODE_INACTIVE
    pose_old = (0.0, 0.0, 0.0)

    waypoint_list = []
    while True:
        # update the LED animations
        leds.update()

        # update the pose estimator
        poseX.update()
        
        # update the motion controller
        (tv, rv) = motionX.update()
        velocity.set_tvrv(tv, rv)

        # update the velocity controller if you are active, otherwise coast so the robot can be pushed
        if mode == MODE_ACTIVE:
            velocity.update()
        else:
            rone.motor_set_pwm('l', 0)
            rone.motor_set_pwm('r', 0)

        # print status every 500ms
        current_time = sys.time()
        if sys.time() > pose_estimator_print_time:
            pose_estimator_print_time += 250
            print 'goal', motionX.get_goal(), 'pose', poseX.get_pose(), 'odo', poseX.get_odometer()
            if mode == MODE_INACTIVE:
                if (math2.pose_subtract(poseX.get_pose(), pose_old) != (0.0, 0.0, 0.0)): 
                    # We're moving!  Yay!  Blink excitedly!
                    leds.set_pattern('r', 'blink_fast', int(LED_BRIGHTNESS * 1.5))
                else:
                    # not moving. sad face.
                    leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            pose_old = poseX.get_pose()

        # check the buttons.  If the red button is pressed, load the waypoint list
        if rone.button_get_value('r'):
            if mode == MODE_INACTIVE:
                poseX.set_pose(0, 0, 0)
                #waypoint_list = [(608, 0), (608, 304), (0, 304), (0, 0)]
                waypoint_list = [(700, 0), (700, 700), (0, 700), (0, 0)]
                mode = MODE_ACTIVE
            
        # check to see if you are at your waypoint.  If so, go to the next one
        if mode == MODE_ACTIVE:
            leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            if motionX.is_done():
                ## Do we have another waypoint?
                if len(waypoint_list) > 0:
                    leds.set_pattern('rgb', 'group', LED_BRIGHTNESS)
                    sys.sleep(250)
                    waypoint = waypoint_list.pop(0)
                    print 'waypoint', waypoint
                    motionX.set_goal(waypoint, MOTION_TV)
                else:
                    print 'waypoint list empty'
                    mode = MODE_INACTIVE
                    velocity.set_tvrv(0, 0)
                    


waypoint_motion()
