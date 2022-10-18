import numpy as np
from constants import GRABBY_HEIGHT, X_CENTRE, Y_CENTRE

def state_detection(box_object, joint_controller_object):
    pass

def block_distances(bl_pos_list, gripper_pos):
    '''
    Calculates the distances (norm) of each fo the blocks from the robot base.
    args: list of x and y coordinates of the blocks
          end effector xy position of the gripper 
    returns: list of block distances
    ''' 
    xg, yg = gripper_pos
    block_distances = []
    for bl_pos in bl_pos_list:
        #unpack position vector into coordinates
        xb, yb = bl_pos
        #calculate the distance
        block_distance = np.hypot((xb-xg),(yb-yg))
        #create list of distances
        block_distances.append(block_distance)
    return block_distances

def block_radius(bl_pos_list):
    '''
    Calculated the radius of the block from the centre of the conveyor belt.
    args: list of x and y coordinates of the blocks
    returns: list of block radii
    '''
    block_radii = []
    for bl_pos in bl_pos_list:
        xb, yb = bl_pos
        rad = np.hypot((xb-X_CENTRE),(yb-Y_CENTRE))
        block_radii.append(rad)
    return block_radii

def grab_pos(x, y):  #matrix in array form
    "x, y are the locations of the block and z is the rotation about z"
    "don't need to worry about z now"
    "also takes charge of what block to pick up"
    return np.array([x, y, GRABBY_HEIGHT])
