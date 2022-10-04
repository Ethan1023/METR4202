# GEOMETRY
L1 = 0.101
L2 = 0.2165-L1
L3 = 0.0945
L4 = 0.083

# TRANSFORMATIONS
T_CAMERA_TO_FIXED = [[ 0.000, -1.000,  0.000, 0.206],
                     [-1.000,  0.000,  0.000, 0.000],
                     [ 0.000,  0.000, -1.000, 0.460],
                     [ 0.000,  0.000,  0.000, 1.000]]

# SAFETY
MAX_JOINT_VEL = (5, 3, 5, 10)
THETA_RANGES = ((-3.1415, 3.1415), (-1.5708, 1.5708), (-1.9, 2.4), (-1.9, 1.9))

# CONTROLLER
ERROR_TOL = 0.1  # get thetas within this angle when moving to a pos

THETA_OFFSET = (0, -0.15, 0, 0.08)  # offset angles to account for strutural sag?

# Highly recommend constant gain across joints 2-4 with no offset
CONTROLLER_GAIN = (2, 10, 10, 10)   # Gain on position error
CONTROLLER_OFFSET = (1, 0, 0, 0)    # Add this to requested velocity

# POSITIONS
# Heights for grabbing a box, when not carrying a box, and when carrying a box respectivly
GRABBY_HEIGHT = 0.03
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



