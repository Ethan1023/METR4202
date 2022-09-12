import numpy as np
from inverse_kinematics import inv_kin
from forward_kinematics import derivePoE, PoE
from modern_robotics import TransToRp
from constants import L1, L2, L3, L4

def main():
    L234 = L2 + L3 + L4
    resolution = 10
    coords = []
    pitches = []
    for x in np.linspace(-L234, L234, resolution):
        for y in np.linspace(-L234, L234, resolution):
            for z in np.linspace(L1-L234, L1+L234, resolution):
                for pitch in np.linspace(-np.pi, np.pi, resolution):
                    coords.append(np.array([x, y, z]))
                    pitches.append(pitch)
    coords = np.array(coords).T
    pitches = np.array(pitches)
    test_kinematics(coords, pitches)

def test_kinematics(coords = None, pitches = None):
    Tsb, S_list = derivePoE()
    if coords is None:
        coords = np.array([-2, 0, 2])
        pitches = 0
    if not hasattr(coords[0], '__iter__'):
        coords = np.array([coords]).T
        pitches = np.array([pitches])
    all_pass = True
    pass_count = 0
    fail_count = 0
    for i in range(len(coords.T)):
        coord = coords.T[i]
        pitch = pitches[i]
        possible = inv_kin(coord, pitch, check_possible=True)
        test_pass = True
        if not possible:
            pass
            #print(f'Not possible to reach!')
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
                print(f'All checks passed for {coord}, {pitch*180/np.pi}!')
                pass_count += 1
        if not test_pass:
            print(f'Checks failed for {coord}, {pitch*180/np.pi}!')
            all_pass = False
            fail_count += 1
    if all_pass:
        print(f'All cases passed! ({pass_count}/{len(coords.T)} possible)')
    else:
        print(f'Some cases failed! ({pass_count}/{len(coords.T)} passed, {fail_count}/{len(coords.T)} failed)')

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
