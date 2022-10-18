import numpy as np
from constants import GRABBY_HEIGHT, X_CENTRE, Y_CENTRE

def state_detection(box_object, joint_controller_object):
    pass

def blocks_from_arm(x, y):
    '''
    Calculates the distances (norm) of each of the blocks from the robot base.
    args: list of x and y coordinates
    returns: list of block distances
    ''' 
    #TO DO: change to calculate xy distance from arm, instead of base
    #this is currently distance of block from the base
    block_distances = []

    #create list of coords
    for i in range(len(x)):
        block_distance = np.hypot(x[i], y[i])
        block_distances.append(block_distance)

    return block_distances

def block_radius(x, y):
    '''
    Calculated the radius of the block from the centre of the conveyor belt.
    args: list of x and y coordinates of the blocks
    returns: list of block radii
    '''
    block_radii = []

    for i in range(len(x))
        rad = np.hypot((x[i]-X_CENTRE),(y[i]-Y_CENTRE))
        block_radii.append(rad)
    return block_radii

def grab_pos(x, y):  #matrix in array form
    '''
    x, y are the locations of the block and z is the rotation about z
    don't need to worry about z now
    also takes charge of what block to pick up
    '''
    return np.array([x, y, GRABBY_HEIGHT])


def which_state():
    
