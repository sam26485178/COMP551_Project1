################################################################################
##                         Waypoint motion Module                             ##
################################################################################

import  math, poseX, velocity

##    NAME
##        motion
##
##    DESCRIPTION
##        Waypoint motion module
##
##    FUNCTIONS
##        init()
##            initialize the _motion_state dictionary
##
##        set_goal(_motion_state, goal_pos)
##            sets the motion state to rotate only
##
##            PARAMS
##                1. _motion_state - the dicitonary motion state
##                2. goal_pos     - (?)
##
##        get_goal(_motion_state)
##            access the motion state dictionary to retrieve the latest goal position
##
##            PARAMS
##                1. _motion_state - the dicitonary motion state
##            RETURN
##                the current goal position stored in the motion state dictionary
##
##        
##        is_done(_motion_state)
##            returns the boolean of _motion_state dictionary's 'motion_done' key
##
##            PARAMS
##                1. _motion_state - the dicitonary motion state
##            RETURN
##                boolean of the state of the motion
##
##        move_to_goal(_motion_state, pose_state, tv_max)
##            moving towards the goal position
##
##            PARAMS
##                1. _motion_state   - the dicitonary motion state
##                2. pose_state     - current pose
##                3. velocity_state - current velocity
##                4. tv_max         - maximum translational velocity
##            RETURN
##                a tuple of the new translational and rotational velocity
        
        
    

MOTION_CAPTURE_DISTANCE = 16
MOTION_RELEASE_DISTANCE = 32
MOTION_CAPTURE_ANGLE = math.pi/2
MOTION_RELEASE_ANGLE = math.pi/10
MOTION_TV_MIN = 20
MOTION_TV_GAIN = 3
MOTION_RV_GAIN = 50
MOTION_RV_MAX = 100

_motion_state = {}
_compute_goal_distance_and_heading = None
_motion_controller_tv = None
_motion_controller_rv = None


def init(compute_goal_distance_and_heading_func, motion_controller_tv_func, motion_controller_rv_func):
    global _motion_state
    global _compute_goal_distance_and_heading
    global _motion_controller_tv
    global _motion_controller_rv
    _motion_state['motion_done'] = True
    _motion_state['rotate_only'] = True
    _motion_state['tv_max'] = 100
    _motion_state['goal_pos'] = (0.0, 0.0)
    _compute_goal_distance_and_heading = compute_goal_distance_and_heading_func
    _motion_controller_tv = motion_controller_tv_func
    _motion_controller_rv = motion_controller_rv_func


def set_goal(goal_pos, tv_max):
    global _motion_state
    _motion_state['motion_done'] = False    
    _motion_state['rotate_only'] = True
    _motion_state['goal_pos'] = goal_pos
    _motion_state['tv_max'] = tv_max


def get_goal():
    global _motion_state
    return _motion_state['goal_pos']


def is_done():
    global _motion_state
    return _motion_state['motion_done']


def update():
    global _motion_state
    global _compute_goal_distance_and_heading
    global _motion_controller_tv
    global _motion_controller_rv
    if _motion_state['motion_done'] == True:
        return (0, 0)
    
    (goal_distance, goal_heading, robot_heading) = _compute_goal_distance_and_heading(_motion_state['goal_pos'], poseX.get_pose())

    if goal_distance < MOTION_CAPTURE_DISTANCE:
        # you are at your destination 
        _motion_state['motion_done'] = True
        tv = 0
        rv = 0
    elif goal_distance > MOTION_RELEASE_DISTANCE:
        _motion_state['motion_done'] = False
        
    if _motion_state['motion_done'] == False:
        # Drive towards goal position
        tv = _motion_controller_tv(goal_distance, _motion_state['tv_max'])

        # Rotate towards goal position
        (rv, heading_error) = _motion_controller_rv(robot_heading, goal_heading)

        if _motion_state['rotate_only']:
            tv = 0
            if abs(heading_error) < MOTION_RELEASE_ANGLE:
                _motion_state['rotate_only'] = False
        else:
            if abs(heading_error) > MOTION_CAPTURE_ANGLE:
                _motion_state['rotate_only'] = True
    return (tv, rv)




