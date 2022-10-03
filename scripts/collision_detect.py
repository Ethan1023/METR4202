'''
detects whether the desired end effector position will result
in a self collision or collision with the test rig

ASSUMPTION: origin is the centre of the belt.
'''
import numpy as np
from joint_controller_ros import JointController
from constants import H_BLOCK, H_BASE, RAD_BELT, H_FENCE, W_FENCE, L_FENCE, L_BASE, W_BASE

#Global variables
p, pitch = JointController.get_current_pos()
x_gripper, y_gripper, z_gripper = p[0], p[1], p[2]
gripper_angle = pitch

block_z = z_gripper - H_BLOCK/2 #height of bottom of the block

def avoids_belt():
    '''
    Checks if the end effector position will result in a collision with the belt.

    Returns: True if no collision, False if collision
    '''
    coll_rad = 1.1*RAD_BELT #collision radius with 10% tolerance

    #use eqn of a circle to determine if the xy coordinates of the end effector are in the area of the belt
    xy_belt = (y_gripper < np.sqrt(coll_rad**2 - x_gripper**2)) and (y_gripper > -(np.sqrt(coll_rad**2 - x_gripper**2)))
    
    if xy_belt and block_z < (H_BLOCK/4):
        return False

    else:
        return True


def avoids_fence():
    '''
    Checks if the end effector position will result in a collision with the fence.
    
    Returns: True if no collision, False if collision
    '''
    y_offset = -(0.1278 + W_FENCE/2) #offset of centre of fence from origin

    fence_x = 1.1*(L_FENCE/2) #absoluate value of collision x dim with 10% tolerance
    fence_y = 1.5*(W_FENCE/2) + y_offset #absoluate value of collision y dim with 50% tolerance
    fence_z = 1.1*H_FENCE #collision z dim with 10% tolerance
  
    xy_fence = (abs(x_gripper) < fence_x) and (abs(y_gripper) < fence_y)

    if xy_fence and block_z < fence_z:
        return False

    else:
        return True


def avoids_base():
    '''
    Checks if the end effector position will result in a collision with the base.
    
    Returns: True if no collision, False if collision
    '''
    y_offset = -0.19 #offset of centre of base from origin

    base_x = 1.1*(L_BASE/2) #absoluate value of collision x dim with 10% tolerance
    base_y = 1.1*(W_BASE/2) + y_offset #absoluate value of collision y dim with 10% tolerance
    base_z = 1.1*H_BASE #collision z dim with 10% tolerance
  
    xy_base = (abs(x_gripper) < base_x) and (abs(y_gripper) < base_y)

    if xy_base and block_z < base_z:
        return False

    else:
        return True


def avoids_blocks():
    '''
    Checks if the end effector position will result in a collision with placed blocks.
    
    Returns: True if no collision, False if collision
    '''
    #TODO: write function
    return True

def valid_gripper_pos():
    '''
    Checks that the gripper's height and angle are valid
    '''
    #Gripper height
    if z_gripper < -(H_BASE - H_BLOCK/2):
        #gripper must never be lower than the height of a block on the ground
        return False
    
    elif z_gripper < H_BLOCK/2: 
        #gripper must be higher than the height of a block on the belt except 
        #when placing a block on the ground
        #TODO: add in AND statement about the gripper not placing a block
        return False

    #Gripper angle
    elif pitch != -np.pi/2: #TODO: condition for if the gripper can't reach the block
        return False

    else:
        return True


def check_collision():
    '''
    Checks that the end effector position will not result in a collision
    (with the belt, fence, base or placed blocks).

    Self collision avoidance is already handled with elbow up and vertical 
    gripper constraints.

    Returns: True if no collision, False if collision
    '''
    no_collisions = avoids_belt() and avoids_fence() and avoids_base() and avoids_blocks() and valid_gripper_pos()

    return no_collisions