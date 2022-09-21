#!/usr/bin/env python3

'''
Use this - https://github.com/UQ-METR4202/dynamixel_interface/blob/master/tutorials/tutorial_1_using_the_controller.md
'''
import rospy
import numpy as np
from modern_robotics import TransToRp
from sensor_msgs.msg import JointState
from inverse_kinematics import inv_kin, atan2
from forward_kinematics import derivePoE, PoE
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
        self.Tsb, self.screws = derivePoE()

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
        #self.joint_pub.publish(joint_state)

    def get_current_pos(self):
        if len(self.joint_names) == 0:
            return None, None
        thetas = []
        for name in self.names:
            index = -1
            for theta, new_name in zip(self.joint_positions, self.joint_names):
                if name == new_name:
                    thetas.append(theta)
        T = PoE(self.Tsb, self.screws, thetas)
        R, p = TransToRp(T)
        pitch = -atan2(R[0], R[2])
        return p, pitch

    def end_effector_publisher(self, desired_coords, desired_pitch, desired_vel=None):
        possible = inv_kin(desired_coords, desired_pitch, check_possible=True)
        print(f'Desired coords = {desired_coords}')
        if possible:
            thetas = inv_kin(desired_coords, desired_pitch)
            print(f'Desired thetas = {thetas}')
            self.joint_state_publisher(thetas, desired_vel)

    def go_to_pos(self, desired_coords, desired_pitch, steps=100):
        current_coords = None

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
    HEIGHT = 0.22
    #desired_coords = [0.1, 0, HEIGHT]
    #desired_pitch = 0
    DIST = 0.1
    desired_coords = [DIST, 0, 0.1]
    desired_pitch = -np.pi/2
    speed = 50
    desired_vel = (speed*np.pi/180, speed*np.pi/180, speed*np.pi/180, speed*np.pi/180)
    #jc.joint_state_publisher(desired_pos, desired_vel)
    jc.end_effector_publisher(desired_coords, desired_pitch, desired_vel)
    rate = rospy.Rate(100)
    i = 0
    while not rospy.is_shutdown():
        #ang = 20
        #desired_pos = ((random.random()-0.5)*ang*np.pi/180, (random.random()-0.5)*ang*np.pi/180, (random.random()-0.5)*ang*np.pi/180, (random.random()-0.5)*ang*np.pi/180)
        #jc.joint_state_publisher(desired_pos, desired_vel)
        #desired_coords = [min(0.2,0.1+i/500), 0, HEIGHT]
        #if i > 50:
        #    desired_coords = [max(0.1,0.2-(i-50)/500), 0, HEIGHT]
        desired_coords = [DIST, 0, max(0, 0.1-i/500)]
        if i > 50:
            desired_coords = [DIST, 0, min(0.1,0+(i-50)/500)]
        jc.end_effector_publisher(desired_coords, desired_pitch, desired_vel)
        jc.get_current_pos()
        print(f'names = {jc.joint_names}')
        print(f'positions = {jc.joint_positions}')
        print(f'velocities = {jc.joint_velocities}')
        print(f'efforts = {jc.joint_efforts}')
        print()
        i += 0.1
        if i > 100:
            i = 0
        rate.sleep()

if __name__ == '__main__':
    main()
