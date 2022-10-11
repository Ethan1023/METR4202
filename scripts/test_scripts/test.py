import numpy as np
from forward_kinematics import derive_inv_jac, calc_frame1_vel, derivePoE, PoE
from inverse_kinematics import inv_kin
from modern_robotics import TransToRp

Tsb, S_list = derivePoE()
Ts3, S_list3 = derivePoE(3)
Ts2, S_list2 = derivePoE(2)
Ts1, S_list1 = derivePoE(1)
Ts0, S_list0 = derivePoE(0)
print(Tsb, S_list)
print(Ts3, S_list3)
print(Ts2, S_list2)
print(Ts1, S_list1)
print(Ts0, S_list0)

thetas = inv_kin(np.array([0.05, 0, 0.05]), -1.5707963267948966)
print(thetas)
#thetas = [0.07165574641618948, 0.772858407774615, 1.699264843583922, 0.7012026613584256]
R, p = TransToRp(PoE(Tsb, S_list, thetas))
R3, p3 = TransToRp(PoE(Ts3, S_list3, thetas))
R2, p2 = TransToRp(PoE(Ts2, S_list2, thetas))
R1, p1 = TransToRp(PoE(Ts1, S_list1, thetas))
R0, p0 = TransToRp(PoE(Ts0, S_list0, thetas))
pitch = np.pi/2 - np.sum(thetas[1:])

print(f'thetas = {thetas}')
print(f'pos = {p}, {pitch}')
print(f'pos3 = {p3}')
print(f'pos2 = {p2}')
print(f'pos1 = {p1}')
print(f'pos0 = {p0}')
exit()
coords_vel = 0.01
coords_diff = np.array([-0.00630008, -0.05978343,  0.00337197])
target_coords_vel = coords_diff / (np.sum(coords_diff**2))**0.5 * coords_vel
target_pitch_vel = 1
J1inv = derive_inv_jac(thetas, printing=False)
L1_4DOF_vel = calc_frame1_vel(thetas, target_coords_vel, target_pitch_vel, printing=False)
target_joint_vel = np.matmul(J1inv, L1_4DOF_vel)

print(f'coords_vel = {target_coords_vel}')
print(f'joint_vel = {target_joint_vel}')
