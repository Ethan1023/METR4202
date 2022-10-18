import numpy as np

def rot(theta, axis):
    '''
    Rotate about axis
    theta - radians to rotate
    axis = x/y/z or 0/1/2 - which axis to rotate about

    returns 3x3 rotation matrix R
    '''
    if axis == 'x' or axis == 0:
        return np.array([[1,              0,              0],
                         [0,              np.cos(theta),  -np.sin(theta)],
                         [0,              np.sin(theta),  np.cos(theta)]])
    if axis == 'y' or axis == 1:
        return np.array([[np.cos(theta),  0,              np.sin(theta)],
                         [0,              1,              0],
                         [-np.sin(theta), 0,              np.cos(theta)]])
    if axis == 'z' or axis == 2:
        return np.array([[np.cos(theta),  -np.sin(theta), 0],
                         [np.sin(theta),  np.cos(theta),  0],
                         [0,              0,              1]])

def yaw_from_quat(quat):
    return np.arctan2(2*(quat.w*quat.z + quat.x*quat.y), 1-2*(quat.y**2+quat.z**2))
