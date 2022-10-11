#!/usr/bin/env python3

'''
Take in desired joint state (position and vel)
Read current joint state (position and vel)
Output joint velocity
'''

import rospy
import time
import numpy as np
from sensor_msgs.msg import JointState
from constants import MAX_JOINT_VEL, THETA_RANGES, THETA_PS, THETA_DS, MIN_VEL

class DesiredStateFilter:
    def __init__(self):
        self.capture_range = 10 * np.pi / 180  # range within velocity direction should be ignored
        self.rate = 200
        self.vel_over_error = 1.5  # Crash if 50% too fast
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

        self.Ps = np.array(THETA_PS)
        self.Ds = np.array(THETA_DS)
        self.max_velocity = np.array(MAX_JOINT_VEL)
        self.stale = False
        print(f'DesiredStateFilter initialised')

    def joint_state_callback(self, joint_state):
        start_time = time.time()
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
                print(f'WARNING - Change in position too high! (dt = {time.time()-self.last_time}s)')
                print(num_vel)
                #self.ERROR = True
        # Check reported velocitiy
        if np.any(np.abs(joint_velocities)/self.vel_over_error > MAX_JOINT_VEL):
            print(f'ERROR - Joint velocity way too high!')
            print(joint_velocities)
            self.ERROR = True
        elif np.any(np.abs(joint_velocities) > MAX_JOINT_VEL):
            print(f'WARNING - Joint velocity too high!')
            print(joint_velocities)
        # Check angle ranges
        for i, theta_range in enumerate(THETA_RANGES):
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
        self.stale = False

    def filter(self, joint_state):
        self.raw_joint_state = joint_state
        self.stale = False
    
    def loop(self):
        rate = rospy.Rate(self.rate)
        while self.raw_joint_state is None and not rospy.is_shutdown():
            rate.sleep()
        last_time = time.time()
        while not rospy.is_shutdown():  # Can run at 200Hz
            while self.stale and not rospy.is_shutdown():
                time.sleep(0.001)
            print(f'{1/(time.time()-last_time)}Hz')
            last_time = time.time()
            #joint_state = self.raw_joint_state
            joint_name = self.raw_joint_state.name
            joint_position = np.array(self.raw_joint_state.position)
            joint_velocity = np.array(self.raw_joint_state.velocity)
            if not self.ERROR:
                # Command velocity + PD controller on position
                #command_velocity = joint_velocity + self.Ds * (joint_velocity - self.joint_velocities)
                command_velocity = joint_velocity.copy()  # Assume derivative is negligible lol - self.joint_velocities is absolute
                #for i in range(len(command_velocity)):  # TODO - This assumes velocity is in requested direction
                #    if not joint_velocity[i] == 0:
                #        command_velocity[i] += self.Ds[i] * (joint_velocity[i] - self.joint_velocities[i]*joint_velocity[i]/abs(joint_velocity[i]))
                if not len(joint_position) == 0:
                    command_velocity += self.Ps * (joint_position - self.joint_positions)
                    #i = 3
                    #print(f'{command_velocity[i]} = {joint_velocity[i]} + {self.Ds[i]} * ({joint_velocity[i]} - {self.joint_velocities[i]}) + {self.Ps[i]} * ({joint_position[i]} - {self.joint_positions[i]})')
                    # Check position in bounds
                #positions = []
                #for i, vel in enumerate(command_velocity):
                #    if (not len(joint_position) == 0) and (abs(vel) < 0.1): # or abs(joint_velocity[i]) == 0):
                #        positions.append(2*self.Ds[i]*joint_velocity[i]/self.Ps[i] + joint_position[i])  # Steady state solution
                #    elif vel > 0:
                #        positions.append(THETA_RANGES[i][1])
                #    else:
                #        positions.append(THETA_RANGES[i][0])
                positions = 2*self.Ds*joint_velocity/self.Ps + joint_position  # Steady state solution for a PD controller - even though D is not implemented, this will cause it to be 'predictive'
                for i, theta_range in enumerate(THETA_RANGES):  # TODO - test
                    if positions[i] < theta_range[0]:
                        print(f'WARNING - Desired theta {i+1} below range')
                        positions[i] = theta_range[0]
                    if positions[i] > theta_range[1]:
                        print(f'WARNING - Desired theta {i+1} above range')
                        positions[i] = theta_range[1]
                #for i in range(len(positions)):
                #    if (positions[i]-self.joint_positions[i]) / command_velocity[i] < 0:
                #        command_velocity[i] = MIN_VEL
                # Scale velocities - TODO is this ok?
                if any(np.abs(command_velocity) > self.max_velocity):
                    command_velocity2 = command_velocity / np.max(np.abs(command_velocity/self.max_velocity)) * 0.999
                    if any(np.abs(command_velocity2) > self.max_velocity):
                        print(f"ERROR - math didn't work")
                        print(command_velocity)
                        print(self.max_velocity)
                        print(command_velocity2)
                        self.ERROR = True
                        command_velocity = (MIN_VEL, MIN_VEL, MIN_VEL, MIN_VEL)
                    else:
                        command_velocity = command_velocity2
                for i, vel in enumerate(command_velocity):
                    if abs(vel) < MIN_VEL:
                        command_velocity[i] = MIN_VEL
                joint_state = JointState()
                joint_state.name = joint_name
                joint_state.position = positions
                joint_state.velocity = command_velocity
                print(f'Commanding position = {positions}')
                print(f'Commanding velocity = {command_velocity}')
                print()
                self.joint_pub.publish(joint_state)
                if self.DEBUG:
                    self.lastPub = joint_state
                    self.lastState = (self.joint_positions, self.joint_velocities, self.joint_efforts)
            else:
                print('ERROR encountered, braking')
                print(self.lastPub)
                print(self.lastState)
                if self.brake_position is None:
                    self.brake_position = self.joint_positions
                joint_state = JointState()
                joint_state.name = self.joint_names
                joint_state.position = self.brake_position
                joint_state.velocity = (MIN_VEL, MIN_VEL, MIN_VEL, MIN_VEL)
                self.joint_pub.publish(joint_state)
                return -1
            self.stale = True
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
