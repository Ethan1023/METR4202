'''
Rules for state machine - figured I might put some here based on assumptions made in collision detect
Never go directly from below belt on left side to right side or vise versa
'''

# GEOMETRY
L1 = 0.101
L2 = 0.2165-L1
L3 = 0.0945
L4 = 0.083
TOT_L = L1+L2+L3+L4

# TRANSFORMATIONS
T_CAMERA_TO_FIXED = [[0.000, 1.000,  0.000, 0.206],
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
THETA_RANGES = ((-3.1415, 3.1415), (-1.5708, 1.5708), (-1.9, 2.4), (-1.9, 1.9))

# CONTROLLER
ERROR_TOL = 0.1  # get thetas within this angle when moving to a pos

THETA_OFFSET = (0.05, -0.15, 0, 0.08)  # offset angles to account for strutural sag?
RAD_OFFSET = -0.03

# Highly recommend constant gain across joints 2-4 with no offset
CONTROLLER_GAIN = (2, 10, 10, 10)   # Gain on position error
CONTROLLER_OFFSET = (1, 0, 0, 0)    # Add this to requested velocity

# POSITIONS
# Heights for grabbing a box, when not carrying a box, and when carrying a box respectivly
GRABBY_HEIGHT = 0.02
EMPTY_HEIGHT = 0.05
CARRY_HEIGHT = 0.08

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
GRAB_TIME = 0.5

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
STATE_NAMES = ['reset', 'find', 'grab', 'colour', 'place', 'error', 'trap', 'toss']

#COLOUR BLOCK LOCATIONS TODO - CHANGE THIS!
PLACE_DICT = {"red": (0, 0)}


# Other?
VELOCITY_AVG_TIME = 0.5  # How long across to average time
#VELOCITY_THRESHOLD = 0.01   # How fast counts as moving
OMEGA_THRESHOLD = 3 * 3.1415 / 30  # How fast counts as moving - tradeoff between how slow counts as moving and sensor noise
