#!/usr/bin/env python3

'''
Use this - https://github.com/UQ-METR4202/dynamixel_interface/blob/master/tutorials/tutorial_1_using_the_controller.md
'''
import rospy
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from metr4202.msg import Pos  # Custom messages from msg/
from inverse_kinematics import inv_kin
from forward_kinematics import derivePoE, PoE
from modern_robotics import TransToRp
from constants import THETA_RANGES, ERROR_TOL, MAX_JOINT_VEL, CONTROLLER_GAIN, CONTROLLER_OFFSET, THETA_OFFSET, GRABBY_HEIGHT, EMPTY_HEIGHT, CARRY_HEIGHT


class JointController:
    '''
    Publish to desired_joint_state
    Subscribe to joint_states
    Implement as action?
    '''
    def __init__(self):
        self.other_init()
        self.rospy_init()
        print(f'JointController initialised')
   
    def rospy_init(self):
        # Create node
        rospy.init_node('joint_controller', anonymous=False)
        # Publish to desired joint states
        #self.joint_pub = rospy.Publisher('desired_joint_states_raw', JointState, queue_size=1)
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Subscribe to actual joint states
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        # Subscribe to requested position
        rospy.Subscriber('desired_pos', Pos, self.end_pos_callback)
        rospy.Subscriber('desired_pose', Pose, self.end_pose_callback)

        while len(self.joint_names) == 0:
            # Block operation until state vector obtained
            time.sleep(0.01)
    
    def other_init(self):
        self.theta_stale = True
        self.pos_stale = True
        # Variables for storing most recent joint state
        # Note that a list in ROS is interpreted as a tuple
        self.joint_names = ()
        self.joint_positions = ()
        self.joint_velocities = ()
        self.joint_efforts = ()
        # Joint names
        self.names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        
        self.max_vel = np.array(MAX_JOINT_VEL)
        self.thetas = None

        self.printing = True
        
        self.desired_coords = None
        self.desired_pitch = None

        self.Tsb, self.screws = derivePoE()

    def run(self):
        while not rospy.is_shutdown():
            while self.pos_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            self.pos_stale = True
            self.go_to_pos(self.desired_coords, self.desired_pitch)

    def end_pos_callback(self, pos):
        self.desired_coords = np.array([pos.x, pos.y, pos.z])
        self.desired_pitch = pos.pitch
        self.pos_stale = False

    def end_pose_callback(self, pose):
        self.desired_coords = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.desired_pitch = -np.pi/2
        self.pos_stale = False

    def joint_state_callback(self, joint_state):
        '''
        Keep actual joint state up to date
        '''
        self.joint_names = joint_state.name  # TODO - make this only be set once, or hard code?
        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_efforts = joint_state.effort
        # Rearrange thetas in order from 1 to 4
        thetas = []
        for i, name in enumerate(self.names):
            for theta, new_name in zip(self.joint_positions, self.joint_names):
                if name == new_name:
                    thetas.append(theta - THETA_OFFSET[i])
        self.thetas = thetas
        self.theta_stale = False

    def joint_state_publisher(self, desired_pos, desired_vel=None):
        '''
        Publish desired joint angles and velocities
        '''
        joint_state = JointState()
        joint_state.name = self.names
        joint_state.position = desired_pos + np.array(THETA_OFFSET)
        # Limit joint angles
        for i in range(len(desired_pos)):
            if desired_pos[i] > THETA_RANGES[i][1]:
                print(f'WARNING - Desired pos too high, saturating')
                desired_pos[i] = THETA_RANGES[i][1]
            elif desired_pos[i] < THETA_RANGES[i][0]:
                print(f'WARNING - Desired pos too high, saturating')
                desired_pos[i] = THETA_RANGES[i][0]
        if desired_vel is not None:
            # Limit velocity
            des_vel = np.array(desired_vel)
            if any(np.abs(des_vel) > self.max_vel):
                desired_vel = np.minimum(self.max_vel, np.abs(des_vel))
                print(f'WARNING - Saturating joint velocity from {des_vel} to {desired_vel}')
            desired_vel = np.maximum(np.abs(desired_vel), np.ones(4)*0.02)
            joint_state.velocity = desired_vel
            if self.printing:
                print(f'Publishing {joint_state.name}, {joint_state.position}, {joint_state.velocity}')
        else:
            if self.printing:
                print(f'Publishing {joint_state.name}, {joint_state.position}')
        self.joint_pub.publish(joint_state)

    def get_current_pos(self, thetas = None):
        '''
        Get current end effector position and pitch
        '''
        if thetas is None: 
            if self.thetas is None: # If joint state has not been published yet, return none
                return None, None
            thetas = self.thetas
        # Obtain end effector configuration
        T = PoE(self.Tsb, self.screws, self.thetas)
        R, p = TransToRp(T)
        pitch = np.pi/2 - np.sum(thetas[1:])
        return p, pitch

    def go_to_pos(self, desired_coords, desired_pitch, gain=None, offset=None):
        '''
        Use gain to set vel?
        '''
        if gain is None:
            gain = np.array(CONTROLLER_GAIN)
        if offset is None:
            offset = np.array(CONTROLLER_OFFSET)
        possible = inv_kin(desired_coords, desired_pitch, check_possible=True) # Check if possible
        if not possible:
            print(f'ERROR - NOT POSSIBLE')
            return False
        desired_thetas = inv_kin(desired_coords, desired_pitch)  # Obtain angles
        while self.theta_stale and not rospy.is_shutdown():  # Wait for new values
            time.sleep(0.01)
        thetas = np.array(self.thetas)
        self.theta_stale = True
        print(f'Desired pos = {desired_coords}, {desired_pitch}')
        error = np.sqrt(np.sum((desired_thetas-thetas)**2))  # Calculate error
        print(f'init error = {error}')
        # TODO - check route is safe and modify as necessary
        # e.g. if height below wheel thing, but end destination in safe area, don't go low until clear of wheel
        #       if current pos above wheel - modify end pos to be safe height
        error_temp = ERROR_TOL
        while error > ERROR_TOL and not rospy.is_shutdown() and self.pos_stale:
            if error_temp < ERROR_TOL:
                print(f'WARNING - Reached temporary waypoint')
            current_pos = self.get_current_pos()
            # Get 'waypoint'
            temp_desired_pos = self.modify_path(current_pos (desired_coords, desired_pitch))
            # Check waypoint is possible
            possible = inv_kin(temp_desired_pos[0], temp_desired_pos[1], check_possible=True)
            if not possible:
                print(f'ERROR - MODIFIED PATH NOT POSSIBLE')
                print(current_pos)
                print((desired_coords, desired_pitch))
                print(temp_desired_pos)
                return False
            # Use temporary waypoint
            temp_desired_thetas = inv_kin(temp_desired_pos[0], temp_desired_pos[1])
            print(f'Current thetas = {thetas}')
            print(f'Desired thetas = {desired_thetas}')
            print(f'Waypoint thetas = {temp_desired_thetas}')
            #thetas_diff = abs(desired_thetas - thetas)  # State error
            thetas_diff = abs(temp_desired_thetas - thetas)  # State error
            print(f'Theta error = {thetas_diff}')
            target_thetas_vel = thetas_diff * gain # P controller
            target_thetas_mag = np.sqrt(np.sum(target_thetas_vel**2))  # Scale velocity back a little - nonlinear P
            target_thetas_vel = target_thetas_vel/target_thetas_mag**0.5 + offset  # Offset by a min velocity
            print(f'Target joint vel = {target_thetas_vel}')
            # Update velocity
            #self.joint_state_publisher(desired_thetas, target_thetas_vel)
            self.joint_state_publisher(temp_desired_thetas, target_thetas_vel)
            # Wait for new values
            while self.theta_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            thetas = np.array(self.thetas)
            self.theta_stale = True
            # Update error
            error = np.sqrt(np.sum((desired_thetas-thetas)**2))
            error_temp = np.sqrt(np.sum((temp_desired_thetas-thetas)**2))
            print(f'loop error = {error}')
            print()
            print()
        return True

    def modify_path(self, current_pos, desired_pos):
        '''
        Accepts current and desired positions
        Returns modified desired position to avoid collisions
        '''
        # TODO - modify combos that are likely to result in a collision
        return desired_pos

def main():
    # Create ROS node
    jc = JointController()
    # Prevent python from exiting
    #test(jc)
    jc.run()
    rospy.spin()

def test(jc):
    while not rospy.is_shutdown():
        height = CARRY_HEIGHT
        width = 0.2
        jc.go_to_pos(np.array([0.1, 0.5*width, height]), -np.pi/2)
        time.sleep(1)
        jc.go_to_pos(np.array([0.1, -0.5*width, height]), -np.pi/2)
        time.sleep(1)
        jc.go_to_pos(np.array([0.17, -0.5*width, height]), -np.pi/2)
        time.sleep(1)
        jc.go_to_pos(np.array([0.17, 0.5*width, height]), -np.pi/2)
        time.sleep(1)

if __name__ == '__main__':
    main()
