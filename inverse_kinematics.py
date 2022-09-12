import numpy as np
from constants import L1, L2, L3, L4

def main():
    coords = np.array([-2, 0, 2])
    pitch = 0*np.pi/180
    inv_kin(coords, pitch, True)
    
    coords_2 = np.array([[3, -2], [0, 0], [1, 2]])
    pitch_2 = np.array([0, 0])
    inv_kin(coords_2, pitch_2)

def inv_kin(coords, pitch, printing=False, self_check = True, check_possible=False):
    '''
    Yaw is dependent on x, y
    Pitch is mostly independent of x, y, z except close to end of reach
    Roll cannot be controlled

    coords - array of x, y, z
    pitch - float - pitch of end effector (0 = horizontal)
    '''
    char = ' '
    if hasattr(coords[0], '__iter__'):
        char = '\n'
    if printing:
        print(f'Given coords ={char}{coords}{char}and pitch ={char}{pitch*180/np.pi}')
    r = (coords[0]**2 + coords[1]**2)**0.5
    th1 = atan2(coords[0], coords[1])
    if printing:
        print(f'Changing coords, r ={char}{r},{char}theta1 ={char}{th1*180/np.pi}')
        print(f'Now joints can be represented as p = (r, z)')

    # By definition
    p4 = np.array([r, coords[2]])
    if printing:
        print(f'End effector, p4 ={char}{p4}')
    # From pitch, work back to p3
    p3 = np.array([p4[0]-np.cos(pitch)*L4, p4[1]-np.sin(pitch)*L3])
    if printing:
        print(f'From pitch, p3 ={char}{p3}')
    # From geometry
    p1 = np.array([0, L1])
    if printing:
        print(f'From geometry, p1 ={char}{p1}')
    # Vector from p1 to p3
    v13 = (p3.T - p1).T
    d13 = np.sum(v13**2,axis=0)**0.5
    if check_possible:
        if d13 > (L2 + L3):
            return False
        return True
    if printing:
        print(f'Distance from p1 to p3 ={char}{d13}')
    # Can find theta3 through supplement of angle found from cosine rule
    # See W6 slide 19 - note: our definition of angles is in opposite direction
    cth3 = (d13**2 - L2**2 - L3**2) / (2*L2*L3)
    th3_1 = -atan2(cth3, (1-cth3**2)**0.5)
    th3_2 = -atan2(cth3, -(1-cth3**2)**0.5)
    if printing:
        print(f'Solutions for theta3 ={char}{th3_1*180/np.pi},{char}{th3_2*180/np.pi}')
    # Find p1, W6 slide 20
    th2_1 = np.pi/2 - atan2(v13[0], v13[1]) + atan2(L2 + L3*np.cos(-th3_1), L3*np.sin(-th3_1))
    th2_2 = np.pi/2 - atan2(v13[0], v13[1]) + atan2(L2 + L3*np.cos(-th3_2), L3*np.sin(-th3_2))
    if printing:
        print(f'From p3, corresponding solutions for theta2 ={char}{th2_1*180/np.pi},{char}{th2_2*180/np.pi}')
    th4_1 = np.pi/2 - pitch - th2_1 - th3_1
    th4_2 = np.pi/2 - pitch - th2_2 - th3_2
    if printing:
        print(f'From pitch, corresponding solutions for theta4 ={char}{th4_1*180/np.pi},{char}{th4_2*180/np.pi}')
    # soln[solution][theta][case]
    if self_check:
        t1 = np.sin(th2_1)*L2 + np.sin(th2_1+th3_1)*L3 + np.sin(th2_1+th3_1+th4_1)*L4 - r
        t2 = np.sin(th2_2)*L2 + np.sin(th2_2+th3_2)*L3 + np.sin(th2_2+th3_2+th4_2)*L4 - r
        t3 = np.cos(th2_1)*L2 + np.cos(th2_1+th3_1)*L3 + np.cos(th2_1+th3_1+th4_1)*L4 + L1 - coords[2]
        t4 = np.cos(th2_2)*L2 + np.cos(th2_2+th3_2)*L3 + np.cos(th2_2+th3_2+th4_2)*L4 + L1 - coords[2]
        t5 = th2_1 + th3_1 + th4_1 + pitch - np.pi/2
        t6 = th2_2 + th3_2 + th4_2 + pitch - np.pi/2
        tall = np.array([t1, t2, t3, t4, t5, t6])
        if not np.all(np.isclose(tall, 0)):
            print('ERROR - Forward kinematics self check failed!')
    return np.array([[th1, th2_1, th3_1, th4_1], [th1, th2_2, th3_2, th4_2]])

def atan2(b, a):
    # Suppress divide by zero warnings
    with np.errstate(divide='ignore'):
        theta = np.arctan(a/b)
    # Add pi if cos < 0, and saturate between -pi to pi
    return loop(theta + np.pi*(b<0))

def loop(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi

if __name__ == '__main__':
    main()
