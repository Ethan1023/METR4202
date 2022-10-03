import numpy as np
from constants import H_BLOCK, H_BASE, RAD_BELT, H_FENCE, W_FENCE, L_FENCE, L_BASE, W_BASE, BASE_TO_BELT

class CollisionHandler:
    '''
    Detects whether the desired end effector position will result
    in a self collision or collision with the test rig

    '''
    def __init__(self):
        self.p = desired_coords
        self.gripper_angle = desired_pitch

        self.x_gripper = p[0]
        self.y_gripper = p[1]
        self.z_gripper = p[2]

        self.z_block = z_gripper - H_BLOCK/2 #height of bottom of the block


    def avoids_belt(self, x_gripper, y_gripper, z_block):
        '''
        Checks if the end effector position will result in a collision with the belt.

        Returns: True if no collision, False if collision
        '''
        belt_rad = 1.1*RAD_BELT #collision radius with 10% tolerance

        #use eqn of a circle to determine if the xy coordinates of the end effector are in the area of the belt
        xy_belt = (x_gripper < (np.sqrt(belt_rad**2 - y_gripper**2) + BASE_TO_BELT)) and (x_gripper > (-(np.sqrt(belt_rad**2 - y_gripper**2)) + BASE_TO_BELT))
        
        if xy_belt and z_block < (H_BLOCK/4):
            return False

        else:
            return True


    def avoids_fence(self, x_gripper, y_gripper, z_block):
        '''
        Checks if the end effector position will result in a collision with the fence.
        
        Returns: True if no collision, False if collision
        '''
        fence_x = 1.1*(W_FENCE + W_BASE/2) #absoluate value of collision x dim with 10% tolerance
        fence_y = 1.1*(L_FENCE/2) #absoluate value of collision y dim with 10% tolerance
        fence_z = 1.1*H_FENCE #collision z dim with 10% tolerance
    
        xy_fence = (x_gripper < fence_x) and (abs(y_gripper) < fence_y)

        if xy_fence and z_block < fence_z:
            return False

        else:
            return True


    def avoids_base(self, x_gripper, y_gripper, z_block):
        '''
        Checks if the end effector position will result in a collision with the base.
        
        Returns: True if no collision, False if collision
        '''

        base_x = 1.1*(L_BASE/2) #absoluate value of collision x dim with 10% tolerance
        base_y = 1.1*(W_BASE/2) #absoluate value of collision y dim with 10% tolerance
        base_z = 0.01 #collision z dim with 1cm tolerance
    
        xy_base = (abs(x_gripper) < base_x) and (abs(y_gripper) < base_y)

        if xy_base and z_block < base_z:
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

    def valid_gripper_pos(self, z_gripper, gripper_angle):
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
        elif gripper_angle != -np.pi/2: #TODO: condition for if the gripper can't reach the block
            return False

        else:
            return True


    def check_collision(self):
        '''
        Checks that the end effector position will not result in a collision
        (with the belt, fence, base or placed blocks).

        Self collision avoidance is already handled with elbow up and vertical 
        gripper constraints.

        Returns: True if no collision, False if collision
        '''
        no_collisions = self.avoids_belt() and self.avoids_fence() and self.avoids_base() and self.avoids_blocks() and self.valid_gripper_pos()

        return no_collisions