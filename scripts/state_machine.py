import numpy as np

def main():
    pass

def block_distances(coords_list):
    '''
    Calculates the distances (norm) of each fo the blocks from the robot base.
    args: list of x and y coordinates
    returns: list of block distances
    '''
    block_distances = []
    for coords in coords_list:
        #unpack position vector into coordinates
        x, y = coords
        #calculate the distance
        block_distance = np.hypot(x,y)
        #create list of distances
        block_distances.append(block_distance)
    return block_distances


if __name__ == "__main__":
    main()