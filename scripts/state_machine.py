import numpy as np
from constants import GRABBY_HEIGHT, X_CENTRE, Y_CENTRE

def blocks_from_arm(x, y):
    '''
    Calculates the distances (norm) of each of the blocks from the robot base.
    args: list of x and y coordinates
    returns: list of block distances from base
    ''' 
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

    for i in range(len(x)):
        rad = np.hypot((x[i]-X_CENTRE),(y[i]-Y_CENTRE))
        block_radii.append(rad)
    return block_radii

def bl_to_bl_dist(xs, ys):
    '''
    Calculates the sum of distances between each block.

    args: list of x and y coords for all blocks located.
    returns: list of cumulative distances to other blocks.
    '''
    total_ds = [] #total distance to other blocks
    for x, y in zip(xs, ys):
        #create lists of all other x and y points
        other_xs = xs.copy()
        other_ys = ys.copy()
        d_list = []
        for xo, yo in zip(other_xs, other_ys):
            #calculate distance from one block to another
            d = np.hypot((x-xo), (y-yo))
            d_list.append(d)
        #add together all the distances to the other points
        total_d = sum(d_list)
        total_ds.append(total_d)
    return total_ds

def grab_pos(x, y):  #matrix in array form
    '''
    x, y are the locations of the block and z is the rotation about z
    don't need to worry about z now
    also takes charge of what block to pick up
    '''
    return np.array([x, y, GRABBY_HEIGHT])
