import numpy as np
from constants import H_BLOCK, H_BASE, RAD_BELT, H_FENCE, W_FENCE, L_FENCE, L_BASE, W_BASE, BASE_TO_BELT, BASE_TO_FENCE


def modify_path(self, current_pos, desired_pos):
    '''
    Accepts current and desired positions
    Returns modified desired position to avoid collisions
    '''
    # TODO - modify combos that are likely to result in a collision
    return desired_pos


class CollisionHandler:
    '''
    Detects whether the desired end effector position will result in a collision

    '''
    def __init__(self, desired_coords, desired_pitch):
        self.update(desired_coords, desired_pitch)

    def update(self, desired_coords, desired_pitch):
        self.p = desired_coords
        self.gripper_angle = desired_pitch

        self.x_gripper = self.p[0]
        self.y_gripper = self.p[1]
        self.z_gripper = self.p[2]

        self.z_block = self.z_gripper - H_BLOCK/2 #height of bottom of the block


    def avoids_belt(self):
        '''
        Checks if the end effector position will result in a collision with the belt.
        Returns: True if no collision, False if collision
        '''
        belt_rad = 1.1*RAD_BELT #collision radius with 10% tolerance

        #use eqn of a circle to determine if the xy coordinates of the end effector are in the area of the belt
        xy_belt = (self.x_gripper < (np.sqrt(belt_rad**2 - self.y_gripper**2) + BASE_TO_BELT)) and\
                  (self.x_gripper > (-(np.sqrt(belt_rad**2 - self.y_gripper**2)) + BASE_TO_BELT))
        
        if xy_belt and self.z_block < (H_BLOCK/4):
            return False
        else:
            return True


    def avoids_fence(self):
        '''
        Checks if the end effector position will result in a collision with the fence.        
        Returns: True if no collision, False if collision
        '''
        fence_x1 = 0.8*(BASE_TO_FENCE - H_BLOCK) #absoluate value of min collision x dim with 10% tolerance
        fence_x2 = 1.1*(W_FENCE + BASE_TO_FENCE) #absoluate value of max collision x dim with 10% tolerance
        fence_y = 1.2*(L_FENCE/2) #absoluate value of collision y dim with 20% tolerance
        fence_z = 1.2*H_FENCE #collision z dim with 10% tolerance
    
        xy_fence = (self.x_gripper < fence_x2) and (self.x_gripper > fence_x1) and\
             (abs(self.y_gripper) < fence_y)

        if xy_fence and self.z_block < fence_z:
            return False
        else:
            return True


    def avoids_base(self):
        '''
        Checks if the end effector position will result in a collision with the base.        
        Returns: True if no collision, False if collision
        '''

        base_x = 1.1*(L_BASE/2) #absoluate value of collision x dim with 10% tolerance
        base_y = 1.1*(W_BASE/2) #absoluate value of collision y dim with 10% tolerance
        base_z = 0.01 #collision z dim with 1cm tolerance
    
        xy_base = (abs(self.x_gripper) < base_x) and (abs(self.y_gripper) < base_y)

        if xy_base and self.z_block < base_z:
            return False
        else:
            return True


    def avoids_blocks(self):
        '''
        Checks if the end effector position will result in a collision with placed blocks.        
        Returns: True if no collision, False if collision
        '''
        #TODO: write function
        return True

    def valid_gripper_pos(self):
        '''
        Checks that the gripper's height and angle are valid
        '''
        #Gripper height
        if self.z_gripper < -(H_BASE - H_BLOCK/2):
            #gripper must never be lower than the height of a block on the ground
            return False        

        #Gripper angle
        elif abs(self.gripper_angle + np.pi/2) > 10*np.pi/180: #TODO: condition for if the gripper can't reach the block
            return False
        else:
            return True


    def avoids_all_collisions(self):
        '''
        Checks that the end effector position will not result in a collision
        (with the belt, fence, base or placed blocks).

        Self collision avoidance is already handled with elbow up and vertical 
        gripper constraints.
        Returns: True if no collision, False if collision
        '''
        no_collisions = self.avoids_belt() and self.avoids_fence() and self.avoids_base() and\
                        self.avoids_blocks() and self.valid_gripper_pos()

        return no_collisions
