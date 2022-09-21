#!/usr/bin/env python3

'''
Use this - https://github.com/UQ-METR4202/dynamixel_interface/blob/master/tutorials/tutorial_1_using_the_controller.md
'''
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from inverse_kinematics import inv_kin
import random

class JointController:
    '''
    Publish to desired_joint_state
    Subscribe to joint_states
    Implement as action?
    '''
    def __init__(self):
        # Create node
        rospy.init_node('joint_controller', anonymous=False)
        # Publish to desired joint states
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Subscribe to actual joint states
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        # Variables for storing most recent joint state
        # Note that a list in ROS is interpreted as a tuple
        self.joint_names = ()
        self.joint_positions = ()
        self.joint_velocities = ()
        self.joint_efforts = ()
        self.names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')

    def joint_state_callback(self, joint_state):
        self.joint_names = joint_state.name  # TODO - make this only be set once, or hard code?
        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_efforts = joint_state.effort

    def joint_state_publisher(self, desired_pos, desired_vel=None):
        joint_state = JointState()
        #joint_state.joint_name = self.joint_names
        joint_state.name = self.names
        joint_state.position = desired_pos
        if desired_vel is not None:
            joint_state.velocity = desired_vel
        self.joint_pub.publish(joint_state)

def main():
    # Create ROS node
    jc = JointController()
    # Prevent python from exiting
    test(jc)
    rospy.spin()

def test(jc):
    print(f'names = {jc.joint_names}')
    print(f'positions = {jc.joint_positions}')
    print(f'velocities = {jc.joint_velocities}')
    print(f'efforts = {jc.joint_efforts}')
    print()
    desired_pos = (0*np.pi/180, 0*np.pi/180, 0*np.pi/180, 0*np.pi/180)
    desired_vel = (20*np.pi/180, 20*np.pi/180, 20*np.pi/180, 20*np.pi/180)
    jc.joint_state_publisher(desired_pos, desired_vel)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        ang = 20
        desired_pos = ((random.random()-0.5)*ang*np.pi/180, (random.random()-0.5)*ang*np.pi/180, (random.random()-0.5)*ang*np.pi/180, (random.random()-0.5)*ang*np.pi/180)
        jc.joint_state_publisher(desired_pos, desired_vel)
        print(f'names = {jc.joint_names}')
        print(f'positions = {jc.joint_positions}')
        print(f'velocities = {jc.joint_velocities}')
        print(f'efforts = {jc.joint_efforts}')
        print()
        rate.sleep()

if __name__ == '__main__':
    main()
