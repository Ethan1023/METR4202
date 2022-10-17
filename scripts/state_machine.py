import numpy as np
import BoxTransform

def block_distances(pos_list):
    '''
    Calculates the distances (norm) of each fo the blocks from the robot base.
    args: pos (4x4 transofrmation matrix)
    returns: list of block distances
    '''
    coords_list = []
    block_distances = []
    for pos in pos_list:
        coords = pos[:3, 2]
        coords_list.append(coords)    
    for coords in coords_list:
        #unpack position vector into coordinates
        x, y = pos
        #calculate the duistance
        block_distance = np.hypot(x,y)
        #create list of norms
        block_distances.append(block_distance)
    return block_distances