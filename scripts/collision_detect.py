import numpy as np
from constants import H_BLOCK, H_BASE, RAD_BELT, H_FENCE, W_FENCE, L_FENCE, L_BASE, W_BASE, BASE_TO_BELT, BASE_TO_FENCE, THETA_BELT, THETA_FENCE, RADIUS_FENCE, GRABBY_HEIGHT


def modify_path(current_pos, desired_pos, printing=True):
    '''
    Accepts current and desired positions
    Returns modified desired position to avoid collisions
    '''
    # TODO - modify combos that are likely to result in a collision
    des_coords, des_pitch = desired_pos
    des_radius = np.sqrt(des_coords[0]**2 + des_coords[1]**2)
    des_theta1 = np.arctan2(des_coords[1], des_coords[0])
    current_coords, current_pitch = current_pos
    cur_radius = np.sqrt(current_coords[0]**2 + current_coords[1]**2)
    cur_theta1 = np.arctan2(current_coords[1], current_coords[0])
    temp_coords = des_coords.copy()
    temp_pitch = des_pitch
    if des_coords[2] < -(H_BASE - H_BLOCK/2):
        des_coords[2] = -(H_BASE - H_BLOCK/2)
    # Collision avoiders can override each other - order is important as behaviour may change.

    # Check not passing straight over belt to other side
    if abs(des_theta1) > THETA_BELT and abs(cur_theta1) > THETA_BELT and des_theta1 * cur_theta1 < 0:
        # Set desired theta1 to zero - also bump up z as following check assumes z at desired position is safe if moving to belt or that current z is safe if desired z is lower
        temp_coords[0] = des_radius
        temp_coords[1] = 0
        temp_coords[2] = H_BLOCK * 3/4
        if printing:
            print(f'TRAJECTORY WARNING')
            print(f'Passing straight over belt ({cur_theta1*180/np.pi} deg to {des_theta1*180/np.pi} deg), adding waypoint at theta1 = 0')

    # Check current pos at lower height
    if des_coords[2] < current_coords[2]:
        # Likely to be triggered when moving from above belt to side to place block
        # Current position and desired height collide with belt
        if not avoids_belt(current_coords * np.array([1, 1, 0]) + des_coords * np.array([0, 0, 1])):
            # Maintain height until clear
            temp_coords = des_coords * np.array([1, 1, 0]) + current_coords * np.array([0, 0, 1])
            if printing:
                print(f'COLLISION AVOIDANCE WARNING')
                print(f'Increasing desired height from {des_coords[2]} to {temp_coords[2]} to avoid potential conveyor collision')
    else:
        # Likely to be triggered when moving from side to above conveyor
        # Current height and desired position collide with belt
        if not avoids_belt(current_coords * np.array([0, 0, 1]) + des_coords * np.array([1, 1, 0])):
            # Don't move in to belt area until high enough
            # Go to desired radius and height but limit angle
            assert abs(des_theta1) < THETA_BELT, 'ERROR in THETA_BELT or belt collision detection (collision predicted when outside THETA_BELT'
            angle2 = np.sign(des_theta1) * THETA_BELT
            temp_coords[0] = des_radius * np.cos(angle2)
            temp_coords[1] = des_radius * np.sin(angle2)
            if printing:
                print(f'COLLISION AVOIDANCE WARNING')
                print(f'Possible to collide with belt. Desired radius and theta1 = {des_radius}, {des_theta1*180/np.pi}')
                print(f'Limiting desired theta1 to {angle2*180/np.pi}')

    # Avoid fence:
    # If one and only one of the thetas are out of the belt area (i.e. action will pass by the cursed fence corner)
    #   If current radius is too low
    #       set desired theta to current and desired radius to acceptable
    #   elif desired radius is too low
    #       set desired radius to acceptable
    if (cur_theta1 - THETA_FENCE) * (THETA_FENCE - des_theta1) > 0:
        if cur_radius < RADIUS_FENCE:  # Crossing fence and current radius too low - collision could happen any second
            temp_coords[0] = RADIUS_FENCE * np.cos(cur_theta1)
            temp_coords[1] = RADIUS_FENCE * np.sin(cur_theta1)
            if printing:
                print(f'COLLISION AVOIDANCE WARNING')
                print(f'Current radius too low, fence collision possible - increasing radius from {cur_radius} to {RADIUS_FENCE} and locking theta1 until clear')
        elif des_radius < RADIUS_FENCE:  # Collision might occur (or might not, especially due to above statement, but we should be proactive)
            temp_coords[0] = RADIUS_FENCE * np.cos(des_theta1)
            temp_coords[1] = RADIUS_FENCE * np.sin(des_theta1)
            if printing:
                print(f'COLLISION AVOIDANCE WARNING')
                print(f'Desired radius too low, fence collision possible - increasing radius from {des_radius} to {RADIUS_FENCE}')
    return (temp_coords, temp_pitch)

def avoids_belt(pos):
    '''
    Checks if the end effector position will result in a collision with the belt.
    Returns: True if no collision, False if collision
    '''
    belt_rad = 1.1*RAD_BELT #collision radius with 10% tolerance
    x_gripper, y_gripper, z_gripper = pos

    #use eqn of a circle to determine if the xy coordinates of the end effector are in the area of the belt
    xy_belt = (x_gripper < (np.sqrt(belt_rad**2 - y_gripper**2) + BASE_TO_BELT)) and\
              (x_gripper > (-(np.sqrt(belt_rad**2 - y_gripper**2)) + BASE_TO_BELT))
    
    if xy_belt and z_gripper < 0: #GRABBY_HEIGHT*0.9: #(z_gripper - H_BLOCK/2) < (H_BLOCK/4):
        return False
    else:
        return True

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
        # if self.z_gripper < -(H_BASE - H_BLOCK/2):
        #     #gripper must never be lower than the height of a block on the ground
        #     return False        

        #Gripper angle
        if abs(self.gripper_angle + np.pi/2) > 10*np.pi/180: #TODO: condition for if the gripper can't reach the block
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
        no_collisions = avoids_belt(self.p) and self.avoids_fence() and self.avoids_base() and\
                        self.avoids_blocks() and self.valid_gripper_pos()

        return no_collisions
