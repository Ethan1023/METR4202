import numpy as np

from constants import GRABBY_HEIGHT

def state_1(coords):  #matrix in array form
    "x, y are the locations of the block and z is the rotation about z"

    return(coords(0), coords(1), GRABBY_HEIGHT)



