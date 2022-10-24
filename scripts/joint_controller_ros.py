#!/usr/bin/env python3

'''
Use this - https://github.com/UQ-METR4202/dynamixel_interface/blob/master/tutorials/tutorial_1_using_the_controller.md
This script holds all the functions for setting the joint angles of the robot
'''

import time

import modern_robotics as mr
import numpy as np
import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from metr4202.msg import Pos, Thetas  # Custom messages from msg/

from collision_detect import CollisionHandler, modify_path
from forward_kinematics import derivePoE, PoE
from inverse_kinematics import inv_kin

from constants import *

class JointController:
    '''
    Publish to desired_joint_state
    Publish current error (to state machine)
    Subscribe to joint_states (from dynamixel)
    Subscribe to desired pos and desired thetas (from state machine)
    Subscribe to desires pose (from tutorial test script)
    '''
    def __init__(self):
        self.other_init()
        self.rospy_init()
        rospy.loginfo(f'JointController initialised')

    def rospy_init(self):
        '''
        Subscribes and publishes to various nodes
        '''
        # Create node
        rospy.init_node('joint_controller', anonymous=False)
        # Publish to desired joint states
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Publish to position error
        self.error_pub = rospy.Publisher('position_error', Float32, queue_size=1)
        # Subscribe to actual joint states
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        # Subscribe to requested end effector position
        rospy.Subscriber('desired_pos', Pos, self.end_pos_callback)
        rospy.Subscriber('desired_pose', Pose, self.end_pose_callback)
        # Subscribe to requested joint angles
        rospy.Subscriber('desired_thetas', Thetas, self.desired_theta_callback)

        while len(self.joint_names) == 0 and not rospy.is_shutdown():
            # Block operation until state vector obtained
            time.sleep(0.01)

    def other_init(self):
        '''
        Sets variables
        '''
        # Keep track of when new values are received
        self.theta_stale = True
        self.pos_stale = True
        self.ang_stale = True
        # Variables for storing most recent joint state
        # Note that a list in ROS is interpreted as a tuple
        self.joint_names = ()
        self.joint_positions = ()
        self.joint_velocities = ()
        self.joint_efforts = ()
        # Joint names
        self.names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')

        # Maximum velocity
        self.max_vel = np.array(MAX_JOINT_VEL)
        # Current joint angles
        self.thetas = None
        # Modify frequency printing
        self.printing = True
        # Desired end effector position 
        self.desired_coords = None
        self.desired_pitch = None
        # Forwards kinematics for collision handler
        self.Tsb, self.screws = derivePoE()
        self.ch = CollisionHandler((0, 0, 0), 0)
        # Error state
        self.ERROR = False

    def run(self):
        '''
        Runs the gripper
        '''
        while not rospy.is_shutdown():
            # Wait for new command
            while self.pos_stale and self.ang_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            if not self.pos_stale:
                self.pos_stale = True
                self.go_to_pos(self.desired_coords, self.desired_pitch)
            elif not self.ang_stale:
                self.ang_stale = True
                self.go_to_thetas(self.desired_thetas)

    def end_pos_callback(self, pos):
        '''
        Receives position commands,
        Sets desired coordinates and pitch to new coordinates
        '''
        self.desired_coords = np.array([pos.x, pos.y, pos.z])
        self.desired_pitch = pos.pitch
        self.pos_stale = False

    def desired_theta_callback(self, ang):
        '''
        Receives joint angle commands
        Sets the desired theta values
        '''
        self.desired_thetas = np.array(ang.thetas)
        self.ang_stale = False

    def end_pose_callback(self, pose):
        '''
        Receives position commands
        Sets the desired end effector coordinates
        '''
        self.desired_coords = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.desired_pitch = -np.pi/2
        self.pos_stale = False

    def joint_state_callback(self, joint_state):
        '''
        Keep actual joint state up to date
        '''
        # Read values from message
        self.joint_names = joint_state.name
        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_efforts = joint_state.effort
        # Rearrange thetas in order from 1 to 4
        thetas = []
        for i, name in enumerate(self.names):
            for theta, new_name in zip(self.joint_positions, self.joint_names):
                if name == new_name:
                    thetas.append(theta)
        self.raw_thetas = np.array(thetas)
        # Apply joint angle offsets
        for i in range(len(thetas)):
            thetas[i] = thetas[i] - THETA_OFFSET[i]
        if thetas[2] < 0:  # Becomes negative if flipped
            for i in range(1, len(thetas)):
                thetas[i] = thetas[i] + 2*THETA_OFFSET[i]
        # Save resulting joint angles            
        self.thetas = thetas
        self.theta_stale = False

    def joint_state_publisher(self, desired_pos, desired_vel=None, flip=False, dooffset=True):
        '''
        Publish desired joint angles and velocities
        '''
        joint_state = JointState()
        joint_state.name = self.names
        joint_state.position = desired_pos
        # apply joint angle offsets
        if dooffset:
            joint_state.position = desired_pos + np.array(THETA_OFFSET)
            if flip:
                joint_state.position[1:] -= 2*np.array(THETA_OFFSET)[1:]
        # Limit joint angles
        for i in range(len(desired_pos)):
            if desired_pos[i] > THETA_RANGES[i][1]:
                rospy.logwarn(f'WARNING - Desired pos too high, saturating')
                desired_pos[i] = THETA_RANGES[i][1]
            elif desired_pos[i] < THETA_RANGES[i][0]:
                rospy.logwarn(f'WARNING - Desired pos too high, saturating')
                desired_pos[i] = THETA_RANGES[i][0]
        if desired_vel is not None:
            # Limit velocity
            des_vel = np.array(desired_vel)
            if any(np.abs(des_vel) > self.max_vel):
                desired_vel = np.minimum(self.max_vel, np.abs(des_vel))
                rospy.logwarn(f'WARNING - Saturating joint velocity from {des_vel} to {desired_vel}')
            desired_vel = np.maximum(np.abs(desired_vel), np.ones(4)*0.02)
            joint_state.velocity = desired_vel
            if self.printing:
                rospy.logdebug(f'Publishing {joint_state.name}, {joint_state.position}, {joint_state.velocity}')
        else:
            if self.printing:
                rospy.logdebug(f'Publishing {joint_state.name}, {joint_state.position}')
        self.joint_pub.publish(joint_state)

    def error_publisher(self, error):
        '''
        Report joint angle error to state machine
        '''
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)

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
        R, p = mr.TransToRp(T)
        pitch = np.pi/2 - np.sum(thetas[1:])
        return p, pitch

    def go_to_pos(self, desired_coords, desired_pitch, gain=None, offset=None):
        '''
        Given desired_coords and desired_pitch, move end effector to these coordinates,
        Report joint angle error so state machine knows when we have arrived
        '''
        rospy.loginfo(f'got_pos({desired_coords}, {desired_pitch})')
        # Read default gain and offset
        if gain is None:
            gain = np.array(CONTROLLER_GAIN)
        if offset is None:
            offset = np.array(CONTROLLER_OFFSET)
        # Check destination is possible
        possible = inv_kin(desired_coords, desired_pitch, check_possible=True) # Check if possible
        if not possible:
            rospy.logerr(f'ERROR - NOT POSSIBLE')
            rospy.logerr(desired_coords, desired_pitch)
            return False
        desired_thetas = inv_kin(desired_coords, desired_pitch)  # Obtain angles
        # Check if theta1 out of range
        flip = False
        if desired_thetas[0] < THETA_RANGES[0][0] or desired_thetas[0] > THETA_RANGES[0][1]:
            # if out of range, do the flip (add/subtract 180 to base and negate all others)
            desired_thetas[0] -= np.pi * desired_thetas[0]/np.abs(desired_thetas[0])
            desired_thetas[1:] *= -1
            flip = True
        while self.theta_stale and not rospy.is_shutdown():  # Wait for new values
            time.sleep(0.01)
        thetas = np.array(self.thetas)
        self.theta_stale = True
        rospy.logdebug(f'Desired pos = {desired_coords}, {desired_pitch}')
        error = np.sqrt(np.sum((desired_thetas-thetas)**2))  # Calculate error
        rospy.logdebug(f'init error = {error}')
        # Error from waypoint
        error_temp = ERROR_TOL
        do = True
        while do or (error > ERROR_TOL and not rospy.is_shutdown() and self.pos_stale):
            do = False
            if error_temp < ERROR_TOL:
                rospy.logdebug(f'WARNING - Reached temporary waypoint')
            current_pos = self.get_current_pos()
            # Get 'waypoint'
            #temp_desired_pos = modify_path(current_pos, (desired_coords, desired_pitch))
            # No waypoint required to tasks at the moment
            temp_desired_pos = (desired_coords, desired_pitch)
            # Check waypoint is possible
            possible = inv_kin(temp_desired_pos[0], temp_desired_pos[1], check_possible=True)
            if not possible:
                rospy.logerr(f'ERROR - MODIFIED PATH NOT POSSIBLE')
                rospy.logdebug(current_pos)
                rospy.logdebug((desired_coords, desired_pitch))
                rospy.logdebug(temp_desired_pos)
                return False
            # Use temporary waypoint
            if flip:
                temp_desired_thetas = desired_thetas
            else:
                temp_desired_thetas = inv_kin(temp_desired_pos[0], temp_desired_pos[1])
                if temp_desired_thetas[0] < THETA_RANGES[0][0] or temp_desired_thetas[0] > THETA_RANGES[0][1]:
                    temp_desired_thetas[0] -= np.pi * temp_desired_thetas[0]/np.abs(temp_desired_thetas[0])
                    temp_desired_thetas[1:] *= -1
            rospy.logdebug(f'Current thetas = {thetas}')
            rospy.logdebug(f'Desired thetas = {desired_thetas}')
            rospy.logdebug(f'Waypoint thetas = {temp_desired_thetas}')
            thetas_diff = abs(temp_desired_thetas - thetas)  # State error
            rospy.logdebug(f'Theta error = {thetas_diff}')
            target_thetas_vel = thetas_diff * gain # P controller
            target_thetas_mag = np.sqrt(np.sum(target_thetas_vel**2))  # Scale velocity back a little - nonlinear P
            target_thetas_vel = target_thetas_vel/target_thetas_mag**0.5 + offset  # Offset by a min velocity
            rospy.logdebug(f'Target joint vel = {target_thetas_vel}')
            # Update velocity
            self.joint_state_publisher(temp_desired_thetas, target_thetas_vel, flip=flip)
            # Wait for new values
            while self.theta_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            thetas = np.array(self.thetas)
            self.theta_stale = True
            # Update error
            error = np.sqrt(np.sum((desired_thetas-thetas)**2))
            error_temp = np.sqrt(np.sum((temp_desired_thetas-thetas)**2))
            # Publsh angle errors to state machine
            self.error_publisher(error)
            rospy.logdebug(f'loop error = {error}')
            rospy.logdebug('')
            rospy.logdebug('')
        self.error_publisher(error)
        return True

    def go_to_thetas(self, desired_thetas, gain=None, offset=None):
        '''
        Given desired_thetas, move end effector to these coordinates,
        Report joint angle error so state machine knows when we have arrived
        gain and offset are proportional and constant components of a velocity controller
        '''
        rospy.loginfo(f'go_to_angles({desired_thetas})')
        # Read default gain and offset
        if gain is None:
            gain = np.array(CONTROLLER_GAIN)
        if offset is None:
            offset = np.array(CONTROLLER_OFFSET)
        while self.theta_stale and not rospy.is_shutdown():  # Wait for new values
            time.sleep(0.01)
        thetas = np.array(self.raw_thetas)
        self.theta_stale = True
        error = np.sqrt(np.sum((desired_thetas-thetas)**2))  # Calculate error
        rospy.logdebug(f'init error = {error}')

        do = True
        # Loop until arrived or new position received
        while do or (error > ERROR_TOL and not rospy.is_shutdown() and self.ang_stale):
            do = False
            rospy.logdebug(f'Current thetas = {thetas}')
            rospy.logdebug(f'Desired thetas = {desired_thetas}')
            thetas_diff = abs(desired_thetas - thetas)  # State error
            rospy.logdebug(f'Theta error = {thetas_diff}')
            target_thetas_vel = thetas_diff * gain # P controller
            target_thetas_mag = np.sqrt(np.sum(target_thetas_vel**2))  # Scale velocity back a little - nonlinear P
            target_thetas_vel = target_thetas_vel/target_thetas_mag**0.5 + offset  # Offset by a min velocity
            rospy.logdebug(f'Target joint vel = {target_thetas_vel}')
            # Update velocity
            self.joint_state_publisher(desired_thetas, target_thetas_vel, dooffset=False)
            # Wait for new values
            while self.theta_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            thetas = np.array(self.raw_thetas)
            self.theta_stale = True
            # Update error
            error = np.sqrt(np.sum((desired_thetas-thetas)**2))
            # Publsh angle errors to state machine
            self.error_publisher(error)
            rospy.logdebug(f'loop error = {error}')
            rospy.logdebug('')
            rospy.logdebug('')
        self.error_publisher(error)
        return True

def main():
    '''
    Main debugging function
    '''
    # Create ROS node
    jc = JointController()
    # Run controller
    jc.run()
    # Prevent python from exiting
    rospy.spin()

def test(jc):
    '''
    Test function
    '''
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
