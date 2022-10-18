import numpy as np

def state_detection(box_object, joint_controller_object):
    pass

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

def state_1(coords):  #matrix in array form
    "x, y are the locations of the block and z is the rotation about z"
    "don't need to worry about z now"
    "also takes charge of what block to pick up"
    return(coords(0), coords(1), GRABBY_HEIGHT)


if __name__ == "__main__":
    main()
