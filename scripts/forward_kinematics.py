import numpy as np
from modern_robotics import RpToTrans, VecTose3, MatrixExp6, TransToRp
from constants import L1, L2, L3, L4

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
