import numpy as np

def main():
    pass

def block_distances(pos_list):
    '''
    Calculates the distances (norm) of each fo the blocks from the robot base.
    args: pos (4x4 transformation matrix)
    returns: list of block distances
    '''
    coords_list = []
    for pos in pos_list:
        #get 2x1 position vector from 4x4 matrix (x and y coords)
        coords = pos[:2, 3]
        coords_list.append(coords) 

    block_distances = []
    for coords in coords_list:
        #unpack position vector into coordinates
        x, y = pos
        #calculate the distance
        block_distance = np.hypot(x,y)
        #create list of distances
        block_distances.append(block_distance)
        
    return block_distances


if __name__ == "__main__":
    main()