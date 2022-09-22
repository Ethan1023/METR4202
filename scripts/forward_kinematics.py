import numpy as np
from modern_robotics import RpToTrans, VecTose3, MatrixExp6, TransToRp, Adjoint
from constants import L1, L2, L3, L4
from inverse_kinematics import atan2

def main():
    pi = np.pi
    thetas = np.array([pi/2, pi/2, 0, 0])
    Tsb, screws = derivePoE()
    print(Tsb)
    T1 = PoE(Tsb, screws, thetas)
    print(np.round(T1,3))

def derivePoE():
    '''
    Obtain zero position end effector configuration and screws
    '''
    R = np.eye(3)
    p = np.array([0, 0, L1+L2+L3+L4])

    Tsb = RpToTrans(R, p)
    #AdTsb = Adjoint(Tsb)
    
    ws_list = np.array([[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0]])
    qs_list = np.array([[0, 0, 0], [0, 0, L1], [0, 0, L1+L2], [0, 0, L1+L2+L3]])

    S_list = constructscrew(ws_list, qs_list)

    return Tsb, S_list

def derive_inv_jac(thetas, printing=True):
    '''
    Derive 4DOF inverse jacobian in frame 1

    PARAMETERS
    thetas - list of current joint angles
    printing - bool of whether to print some working

    RETURNS
    Inverse 4DOF jacobian in L1 frame (multiply by 4DOF velocities in L1 frame to get joint velocities)
    '''
    # total rotation angles
    theta1 = thetas[0]
    theta2 = thetas[1]
    theta3 = thetas[2] + theta2  # The lack of this theta2 cost me so much time and caused great pain
    
    # Calculate joint positions
    pos_list = [np.array([0, 0])]
    pos_list.append(np.array([0, L1]))
    pos_list.append(pos_list[-1] + np.array([np.sin(theta2)*L2, np.cos(theta2)*L2]))
    pos_list.append(pos_list[-1] + np.array([np.sin(theta3)*L3, np.cos(theta3)*L3]))
    qs2d_list = np.array(pos_list)
    qs_list = np.array([qs2d_list.T[0]*np.cos(theta1), qs2d_list.T[0]*np.sin(theta1), qs2d_list.T[1]]).T
    # Calculate scres
    ws_list = np.array([[0, 0, 1], [-np.sin(theta1), np.cos(theta1), 0], [-np.sin(theta1), np.cos(theta1), 0], [-np.sin(theta1), np.cos(theta1), 0]])
    S_list = constructscrew(ws_list, qs_list)

    # Transform from space frame to L1 frame
    Ts1 = PoE(np.eye(4), [np.array([0, 0, 1, 0, 0, 0])], [theta1])

    # By definition
    Js = np.array(S_list).T
    if printing:
        print(f'Js=\n{np.round(Js,3)}')
    # Transform Jacobian to L1 frame
    AdTs1 = Adjoint(Ts1)
    J1 = np.matmul(np.linalg.inv(AdTs1), Js)
    if printing:
        print(f'J1=\n{np.round(J1,3)}')
    # Check it is impossible to generate velocities about x or in y
    if not np.isclose(np.sum(np.abs(J1[0])), 0):
        print(f'WARNING - Impossibly able to rotate about x in L1 frame')
    if not np.isclose(np.sum(np.abs(J1[4])), 0):
        print(f'WARNING - Impossibly able to move in y in L1 frame')
    # Remove rotation about x (roll) and movement in y (yaw-ish)
    J14DOF = np.concatenate([J1[1:4], [J1[5]]])  
    if printing:
        print(f'J14DOF=\n{J14DOF}')
    # Invert so joint velocities can be found
    J1inv = np.linalg.inv(J14DOF)
    if printing:
        print(f'J1inv=\n{J1inv}')
    return J1inv

def calc_frame1_vel(thetas, coords_vel, pitch_vel, printing=True, Tsb=None, screws=None, T04=None, ignore=True):
    '''
    Calculate 4DOF velocity in frame 1 (after first joint)

    PARAMETERS
    thetas - current joint angles
    coords_vel - linear velocity of end effector in space frame
    pitch_vel - angular pitch velocity of end effector (negative of y angular velocity in body frame)
    printing - whether to print working
    Tsb - optionally include zero position end effector configuration
    screws - optionally include screws
    T04 - optionally include end effector configuration
    ignore - whether to ignore impossible velocities or to crash
    '''
    # Obtain optional arguments if they are not supplied
    if Tsb is None or screws is None:
        Tsb, screws = derivePoE()
    if T04 is None:
        T04 = PoE(Tsb, screws, thetas)
    R, p = TransToRp(T04)   # Current end effector configuration
    # Obtain linear velocity through rotation of end effector velocity to end effector frame
    body_lin_vel = np.matmul(np.linalg.inv(R), coords_vel)
    # Radius of end effector from z axis of space frame
    r = (p[0]**2 + p[1]**2)**0.5
    # Angular velocity about z axis of space frame
    omega = body_lin_vel[1] / r
    # Rotate velocity about z to end effector frame using pitch angle,
    # pitch velocity is simply about y in end effector frame
    pitch = np.pi/2 - np.sum(thetas[1:])
    body_ang_vel = np.array([-np.cos(pitch)*omega, -pitch_vel, np.sin(pitch)*omega])
    # Full 6DOG velocity of body
    body_6DOF_vel = np.concatenate([body_ang_vel, body_lin_vel])
    if printing:
        print(f'body_vel={np.round(body_6DOF_vel,3)}')
    T14 = PoE(Tsb, screws[1:], thetas[1:])  # Transform from L1 to L4 (end effector)
    if printing:
        print(f'T14=\n{np.round(T14,3)}')
    # Transform end effector velocity to L1 frame
    AdT14 = Adjoint(T14)
    L1_6DOF_vel = np.matmul(AdT14, body_6DOF_vel)
    if printing:
        print(f'L1_vel={np.round(L1_6DOF_vel,3)}')
    # Check angular velocity about x and velocity in y are zero
    if not np.isclose(L1_6DOF_vel[0], 0):
        print(f'WARNING - Impossible movement about x in L1 frame')
        print(f'    vel={L1_6DOF_vel[0]} rad/s')
        print(f'    L1_ang_vel={L1_6DOF_vel[0:3]}')
        print(f'    body_vel={body_ang_vel}')
        print(f'    thetas={thetas*180/np.pi}')
        print(f'    T14=\n{T14}')
        print(f'    pitch={pitch}')
        if not ignore:
            exit()
    if not np.isclose(L1_6DOF_vel[4], 0):
        print(f'WARNING - Impossible movement in y in L1 frame')
        print(f'    vel={L1_6DOF_vel[4]} m/s')
        if not ignore:
            exit()
    # Remove zero velocities and return 4DOF velocity
    L1_4DOF_vel = np.concatenate([L1_6DOF_vel[1:4], [L1_6DOF_vel[5]]])
    return L1_4DOF_vel

def constructscrew(w, q, v=None):
    '''
    Create screw from rotation w, displacement q and optional linear velocity v
    '''
    noiter = False
    if not hasattr(w, '__iter__'):
        w = [w]
        q = [q]
        v = [v]
        noiter = True
    if v is None:
        v = [None for x in range(len(w))]
    S = []
    for i in range(len(w)):
        if v[i] is None:
            v[i] = -np.cross(w[i], q[i])
        else:
            v[i] = -np.cross(w[i], q[i]) + v[i]
        S.append(np.r_[w[i], v[i]])
    if noiter:
        return S[0]
    return S

def PoE(M, screws, thetas):
    '''
    Calculate final configuration of PoE model

    M - 4x4 matrix of initial configuration
    screws - list of 6 vectors of screw axes
    thetas - list of distances to travel along screw axes
    returns 4x4 matrix of final configuration
    '''
    M = np.matrix.copy(M)  # Copy (actually not needed I think)
    for i in range(len(screws)-1, -1, -1):  # iterate from final screw to initial
        S = screws[i]
        theta = thetas[i]
        V = S * theta           # Find twist
        V_se3 = VecTose3(V)     # Convert to se3
        T = MatrixExp6(V_se3)   # Take matrix exponential
        M = np.matmul(T, M)     # pre-multiply M
    return M  # Return resulting matrix

if __name__ == '__main__':
    main()
