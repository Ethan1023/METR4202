import numpy as np
from inverse_kinematics import inv_kin
from forward_kinematics import derivePoE, PoE
from modern_robotics import TransToRp

def main():
    test_kinematics()

def test_kinematics(coords = None, pitches = None):
    Tsb, S_list = derivePoE()
    if coords is None:
        coords = np.array([-2, 0, 2])
        pitches = 0
    if not hasattr(coords[0], '__iter__'):
        coords = np.array([coords]).T
        pitches = np.array([pitches])
    all_pass = True
    for i in range(len(coords.T)):
        coord = coords.T[i]
        pitch = pitches[i]
        possible = inv_kin(coord, pitch, check_possible=True)
        test_pass = True
        if not possible:
            print(f'Not possible to reach!')
        else:
            soln = inv_kin(coord, pitch)
            T_1 = PoE(Tsb, S_list, soln[0])
            T_2 = PoE(Tsb, S_list, soln[1])
            if not np.all(np.isclose(T_2-T_2, 0)):
                print(f'ERROR - Solutions not identical!')
                test_pass = False
            R, p = TransToRp(T_1)
            if not np.all(np.isclose(coord - p, 0)):
                print(f'ERROR - Location incorrect!')
                test_pass = False
            R2 = np.matmul(rot(soln[0][0], 'z'), rot(np.pi/2-pitch, 'y'))
            if not np.all(np.isclose(R, R2, 0)):
                print(f'ERROR - Orientation incorrect!')
                test_pass = False
            if test_pass:
                print('All checks passed!')
        if not test_pass:
            all_pass = False
    if all_pass:
        print(f'All cases passed!')

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


if __name__ == '__main__':
    main()
