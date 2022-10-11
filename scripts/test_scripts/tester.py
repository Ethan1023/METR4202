import numpy as np
from inverse_kinematics import inv_kin, atan2
from forward_kinematics import derivePoE, PoE, derive_inv_jac, calc_frame1_vel
from modern_robotics import TransToRp
from constants import L1, L2, L3, L4

def main():
    test_many_jacobian()
    #test_all_kinematics()

def test_many_jacobian():
    '''
    Compare jacobian and numerical methods with same initial positions and velocities through numerical integration
    '''
    thetas = np.array([0, 0, np.pi/2, 0])
    thetas = inv_kin(np.array([0.05, 0, 0.1]), -np.pi/2)
    print(thetas*180/np.pi)
    coords_vel = np.array([0.1, 0.1, 0.1])
    #coords_vel = np.array([0, 0, 0])
    #coords_vel = np.array([0, 0, 0])
    #pitch_vel = 1
    pitch_vel = 0.1
    dt = 0.001
    time = 0.1
    Tsb, screws = derivePoE()
    thetas_analytical = thetas
    thetas_numerical = thetas
    print(f'Original thetas = {np.round(thetas*180/np.pi,3)}')
    steps = int(time/dt)
    print(steps)
    thetas_actual = apply_velocity_numerical(thetas, coords_vel, pitch_vel, time, Tsb=Tsb, screws=screws)
    for i in range(steps):
        thetas_analytical = apply_velocity_jacobian(thetas_analytical, coords_vel, pitch_vel, dt, ignore=True, Tsb=Tsb, screws=screws, printing=False)
        thetas_numerical = apply_velocity_numerical(thetas_numerical, coords_vel, pitch_vel, dt, Tsb=Tsb, screws=screws, printing=False)
        if i % int(steps/10) == 0:
            error = np.sum(np.abs(thetas_analytical-thetas_numerical))
            print(f'{i/steps*100}%, error={error*180/np.pi}')
    print(f'Analytical new thetas = {np.round(thetas_analytical*180/np.pi,3)}')
    print(f'Numerical new thetas = {np.round(thetas_numerical*180/np.pi,3)}')
    print(f'Actual new thetas = {np.round(thetas_actual*180/np.pi,3)}')
    error = np.sum(np.abs(thetas_analytical-thetas_numerical))
    print(f'Cumulative absolute error = {error * 180/np.pi} deg')
    print(f'This should be constant with dt as dt -> infinity:')
    print(f'Cumulative absolute error per dt = {error/dt * 180/np.pi} deg/s')
        

def apply_velocity_jacobian(thetas=None, coords_vel=None, pitch_vel=1.0, dt=0.001, printing=False, ignore=True, Tsb=None, screws=None):
    '''
    Update joint angles through jacobian
    '''
    if thetas is None:
        thetas = np.array([0, 0, np.pi/2, 0])
    if coords_vel is None:
        coords_vel = np.array([1, 1, 1])
    if Tsb is None or screws is None:
        Tsb, screws = derivePoE()
    T04 = PoE(Tsb, screws, thetas)  # Find end effector configuration
    if printing:
        print(f'T04=\n{np.round(T04,3)}')
    J1inv = derive_inv_jac(thetas, printing=False)  # Find inverse jacobian in L1 space
    if printing:
        print(f'J1inv=\n{np.round(J1inv,3)}')
    L1_4DOF_vel = calc_frame1_vel(thetas, coords_vel, pitch_vel, Tsb=Tsb, screws=screws, T04=T04, printing=False, ignore=ignore)  # Find velocity in L1 space
    if printing:
        print(f'L1_4DOF_vel={np.round(L1_4DOF_vel,3)}')
    thetas_vel = np.matmul(J1inv, L1_4DOF_vel)  # Find angular velocities
    if printing:
        print(f'Joint vel={np.round(thetas_vel,3)}')
    thetas_2_jac = thetas + thetas_vel * dt

    return thetas_2_jac

def apply_velocity_numerical(thetas=None, coords_vel=None, pitch_vel=1.0, dt=0.001, printing=False, Tsb=None, screws=None):
    '''
    Update joint angles 'numerically'
    '''
    if thetas is None:
        thetas = np.array([0, 0, np.pi/2, 0])
    if coords_vel is None:
        coords_vel = np.array([1, 1, 1])
    if Tsb is None or screws is None:
        Tsb, screws = derivePoE()
    T04 = PoE(Tsb, screws, thetas)
    if printing:
        print(f'T04=\n{np.round(T04,3)}')

    _, coords = TransToRp(T04)  # original coords
    pitch = np.pi/2 - np.sum(thetas[1:])  # original pitch

    coords_2 = coords + coords_vel * dt
    pitch_2 = pitch + pitch_vel * dt
    thetas_2_num = inv_kin(coords_2, pitch_2)
    if printing:
        print(f'Numerical joint vel={np.round((thetas_2_num-thetas)/dt,3)}')
    
    return thetas_2_num


def test_all_kinematics():
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
