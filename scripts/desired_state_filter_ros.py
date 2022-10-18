#!/usr/bin/env python3

'''
Basically a safety / sanity check node
Take in desired joint state
apply position and velocity saturation
pass on to dynamixel controller
'''

import rospy
import time
import numpy as np
from sensor_msgs.msg import JointState
from constants import MAX_JOINT_VEL, THETA_RANGES

class DesiredStateFilter:
    def __init__(self):
        self.capture_range = 10 * np.pi / 180  # range within velocity direction should be ignored
        self.rate = 200
        self.DEBUG = True
        if self.DEBUG:
            self.lastPub = JointState()
            self.lastState = JointState()

        # Create node
        rospy.init_node('desired_state_filter', anonymous=False)
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        # Variables for storing most recent joint state
        # Note that a list in ROS is interpreted as a tuple
        self.joint_names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        self.joint_positions = ()
        self.joint_velocities = ()
        self.joint_efforts = ()
        self.last_time = time.time()
        while len(self.joint_names) == 0:
            # Block operation until state vector obtained
            time.sleep(0.01)
        # Subscribe to raw desired joint states
        rospy.Subscriber('desired_joint_states_raw', JointState, self.filter)
        # Publish to desired joint states
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        self.ERROR = False
        self.brake_position = None
        self.raw_joint_state = None 
        print(f'DesiredStateFilter initialised')

    def joint_state_callback(self, joint_state):
        # Sort values
        transform = np.zeros([4,4])
        for i in range(4):
            transform[i][joint_state.name.index(self.joint_names[i])] = 1
        # New values
        joint_positions = np.matmul(transform, np.array(joint_state.position))
        joint_velocities = np.matmul(transform, np.array(joint_state.velocity))
        joint_efforts = np.matmul(transform, np.array(joint_state.effort))
        # Check change in position
        if len(self.joint_positions) > 0:
            num_vel = (joint_positions - self.joint_positions) / (time.time() - self.last_time)
            if np.any(np.abs(num_vel) > MAX_JOINT_VEL):
                print(f'ERROR - Change in position too high! (dt = {time.time()-self.last_time}s)')
                print(num_vel)
                self.ERROR = True
        # Check reported velocitiy
        if np.any(np.abs(joint_velocities) > MAX_JOINT_VEL):
            print(f'ERROR - Joint velocity too high!')
            print(joint_state.velocity)
            self.ERROR = True
        # Check angle ranges
        for i, theta_range in enumerate(THETA_RANGES):  # TODO - test
            if joint_positions[i] < theta_range[0]-(theta_range[1]-theta_range[0])*0.05:
                print(f'ERROR - {self.joint_names[i]} below range: {joint_positions[i]} < {theta_range[0]}')
                self.ERROR = True
            if joint_positions[i] > theta_range[1]+(theta_range[1]-theta_range[0])*0.05:
                print(f'ERROR - {self.joint_names[i]} above range: {joint_positions[i]} > {theta_range[1]}')
                self.ERROR = True
        # Save new values
        self.joint_positions = joint_positions
        self.joint_velocities = joint_velocities
        self.joint_efforts = joint_efforts
        self.last_time = time.time()

    def filter(self, joint_state):
        self.raw_joint_state = joint_state
    
    def loop(self):
        rate = rospy.Rate(self.rate)
        while self.raw_joint_state is None and not rospy.is_shutdown():
            rate.sleep()
        while not rospy.is_shutdown():
            #joint_state = self.raw_joint_state
            joint_name = self.raw_joint_state.name
            joint_position = np.array(self.raw_joint_state.position)
            joint_velocity = np.array(self.raw_joint_state.velocity)
            print(f'{np.array(joint_position)*180/np.pi - np.array(self.joint_positions)*180/np.pi} = {np.array(joint_position)*180/np.pi} - {np.array(self.joint_positions)*180/np.pi}')
            if not self.ERROR:
                # Set position to end of range if length is zero
                if len(joint_position) == 0:
                    position = []
                    for i, theta_range in enumerate(THETA_RANGES):
                        if joint_velocity[i] < 0:
                            position.append(theta_range[0])
                        else:
                            position.append(theta_range[1])
                    joint_position = position
                else:  # Change position if wrong direction
                    for i in range(len(joint_position)):
                        dist = joint_position[i] - self.joint_positions[i]  # distance to destination
                        print(f'{dist*180/np.pi} = {joint_position[i]*180/np.pi} - {self.joint_positions[i]*180/np.pi}')
                        if abs(dist) > self.capture_range and dist/joint_velocity[i]:  # if distance large and in wrong direction
                            if joint_velocity[i] < 0:  # Change it!
                                joint_position[i] = THETA_RANGES[i][0]
                            else:
                                joint_position[i] = THETA_RANGES[i][1]
                # Set velocity very low if zero -- IMPORTANT - appears to go full speed ahead if speed set below 0.012
                for i in range(len(joint_velocity)):
                    if abs(joint_velocity[i]) < 0.02:
                        joint_velocity[i] = 0.02
                else:
                    # Check position in bounds
                    for i, theta_range in enumerate(THETA_RANGES):  # TODO - test
                        if joint_position[i] < theta_range[0]:
                            print(f'WARNING - Desired theta {i+1} below range')
                            joint_position[i] = theta_range[0]
                        if joint_position[i] > theta_range[1]:
                            print(f'WARNING - Desired theta {i+1} above range')
                            joint_position[i] = theta_range[1]
                # Check velocity
                if any((joint_position-self.joint_positions)/joint_velocity < 0):
                    print(f'WARNING - Desired velocity in opposite direction to desired position')
                joint_state = JointState()
                joint_state.name = joint_name
                joint_state.position = joint_position
                joint_state.velocity = joint_velocity
                self.joint_pub.publish(joint_state)
                if self.DEBUG:
                    self.lastPub = joint_state
                    self.lastState = (self.joint_positions, self.joint_velocities, self.joint_efforts)
            else:
                print('ERROR encountered, braking')
                print(self.lastPub)
                print(self.lastState)
                joint_state = JointState()
                joint_state.name = self.joint_names
                if self.brake_position is None:
                    self.brake_position = self.joint_positions
                joint_state.position = self.brake_position
                joint_state.velocity = (0.02, 0.02, 0.02, 0.02)
                self.joint_pub.publish(joint_state)
                return -1
            rate.sleep()

def main():
    # Create ROS node
    dsf = DesiredStateFilter()
    dsf.loop()
    print(f'Control loop broken')
    # Prevent python from exiting
    rospy.spin()

if __name__ == '__main__':
    main()
