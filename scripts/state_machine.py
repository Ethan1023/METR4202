import numpy as np

def main():
    pass

def block_distances(pos_list):
    '''
    Calculates the distances (norm) of each fo the blocks from the robot base.
    args: pos (4x4 transofrmation matrix)
    returns: list of block distances
    '''
    coords_list = []
    block_distances = []
    for pos in pos_list:
        #get 2x1 position vector from 4x4 matrix (x and y coords)
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


if __name__ == "__main__":
    main()