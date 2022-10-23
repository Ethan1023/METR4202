'''
Rules for state machine - figured I might put some here based on assumptions made in collision detect
Never go directly from below belt on left side to right side or vise versa
'''

import numpy as np

# Throw?
TOSS = False
POWER_LEVEL = 2

# GEOMETRY
L1 = 0.101
L2 = 0.2165-L1
L3 = 0.0945
L4 = 0.083
TOT_L = L1+L2+L3+L4
GRAB_RANGE = L2 + L3

# TRANSFORMATIONS
T_CAMERA_TO_FIXED = [[0.000, 1.000,  0.000, 0.190],
                     [1.000, 0.000,  0.000, 0.000],
                     [0.000, 0.000, -1.000, 0.460],
                     [0.000, 0.000,  0.000, 1.000]]

# NEUTRAL ARM POSITION (M)
WAIT_POSITION = [[1.000, 0.000,  0.000, 0.000],
                 [0.000, 1.000,  0.000, 0.000],
                 [0.000, 0.000,  1.000, TOT_L],
                 [0.000, 0.000,  0.000, 1.000]]

# SAFETY
MAX_JOINT_VEL = (5, 3, 5, 10)
THETA_RANGES = ((-2.5, 2.5), (-1.5708, 1.5708), (-1.9, 2.4), (-1.9, 1.9))

# CONTROLLER
ERROR_TOL = 0.1  # get thetas within this angle when moving to a pos
ERROR_TOL_COARSE = 0.5  # get thetas within this angle when moving to a pos

THETA_OFFSET = (-0.02, -0.1, 0, 0.08)  # offset angles to account for strutural sag?
RAD_OFFSET = -0.015

# Highly recommend constant gain across joints 2-4 with no offset
CONTROLLER_GAIN = (2, 10, 10, 10)   # Gain on position error
CONTROLLER_OFFSET = (1, 0, 0, 0)    # Add this to requested velocity

# POSITIONS
# Heights for grabbing a box, when not carrying a box, and when carrying a box respectivly
COLOUR_DETECT_HEIGHT = 0.25
GRABBY_HEIGHT = 0.02 
EMPTY_HEIGHT = 0.05
CARRY_HEIGHT = 0.08
DROPOFF_HEIGHT = -0.01

#TEST EQUIPMENT DIMENSIONS
H_BLOCK = 0.032
RAD_BELT = 0.12
L_BASE = 0.29
W_BASE = 0.2
H_BASE = 0.05
L_FENCE = 0.2
W_FENCE = 0.005
H_FENCE = 0.05
BASE_TO_BELT = 0.19
BASE_TO_FENCE = 0.0622

#COORDINATES OF CENTRE OF TURNTABLE
X_CENTRE = 0.019
Y_CENTRE = 0

#TIME CONSTANTS
GRAB_TIME = 0.3
COLOUR_CHECK_TIME = 1
MAX_BLOCK_AGE = 5
COLOUR_CHECK_SAMPLES = 10
PREDICT_TIME = 2.5
GRAB_EARLY_TIME = 1.3

THETA_BELT = 0.7  # If theta1 is outside of this range, belt collision should be impossible - TODO
THETA_FENCE = 1  # If theta1 will cross this range, fence collision avoidance will activates - TODO
RADIUS_FENCE = 0.14  # If larger than this radius, fence colliusions should be impossible - TODO

#STATE CONSTANTS
STATE_RESET = 0
STATE_FIND = 1
STATE_GRAB = 2
STATE_COLOUR = 3
STATE_PLACE = 4
STATE_ERROR = 5
STATE_TRAP = 6
STATE_TOSS = 7
STATE_GRAB_MOVING = 8
STATE_FIND_TOSS = 9
STATE_NAMES = ['reset', 'find', 'grab', 'colour', 'place', 'error', 'trap', 'toss', 'grab_moving', 'find_toss']

# ROBOT POSITIONS
POSITION_IDLE = (L4, 0, L1 + L2 + L3)
POSITION_COLOUR_DETECT = (BASE_TO_BELT + 0.02, 0, COLOUR_DETECT_HEIGHT)
POSITION_INTERMEDIATE = (0.1, 0, 0.12)

# BLOCK DROPOFF ZONES
DROPOFF_ZONE = {
    1: (-0.05,  0.15),
    2: (-0.17,  0.05),
    3: (-0.17, -0.05),
    4: (-0.05, -0.15),
}

DROPOFF_POSITION = {
    'red': DROPOFF_ZONE[1],
    'green': DROPOFF_ZONE[2],
    'blue': DROPOFF_ZONE[3],
    'yellow': DROPOFF_ZONE[4],
}

YEET_ANGLE = {
    'red': np.arctan2(DROPOFF_ZONE[1][1], DROPOFF_ZONE[1][0]),
    'green': np.arctan2(DROPOFF_ZONE[2][1], DROPOFF_ZONE[2][0]),
    'blue': np.arctan2(DROPOFF_ZONE[3][1], DROPOFF_ZONE[3][0]),
    'yellow': np.arctan2(DROPOFF_ZONE[4][1], DROPOFF_ZONE[4][0]),
}


# Other?
rpm2rad = lambda w: w * np.pi / 30

VELOCITY_AVG_TIME = 1.0  # uses timestamps this many seconds apart to calculate velocity
VELOCITY_DET_TIME = 0.5  # uses timestamps this many seconds apart to detect velocity
#VELOCITY_AVG_TIME = 2.5  # uses timestamps this many seconds apart to calculate velocity
#VELOCITY_THRESHOLD = 0.01   # How fast counts as moving
OMEGA_THRESHOLD = rpm2rad(0.5) # [rad/s], how fast counts as moving - tradeoff between how slow counts as moving and sensor noise
TASK3B_THRESHOLD = 1.5 # If the belt stops for less than this time, we are in task 3b
ZROT_LIMIT = 30*np.pi/180 #if block is rotatted more than 30 degrees don't try and grab it
