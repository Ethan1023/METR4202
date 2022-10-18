#!/usr/bin/env python3
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from forward_kinematics import derivePoE, PoE
from collision_detect import CollisionHandler
from modern_robotics import TransToRp
from constants import THETA_OFFSET

class CollisionDetectTester:
    def __init__(self):
        self.theta_stale = True
        self.joint_names = ()
        self.joint_positions = ()
        self.joint_velocities = ()
        self.joint_efforts = ()
        self.names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')

        # Create node
        rospy.init_node('collision_detect_tester', anonymous=False)
        # Subscribe to actual joint states
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)

        self.coll_hand = CollisionHandler((0, 0, 0), 0)
        self.Tsb, self.screws = derivePoE()

        while len(self.joint_names) == 0:
            # Block operation until state vector obtained
            time.sleep(0.01)
    
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

    def get_current_pos(self):
        '''
        Get current end effector position and pitch
        '''
        # If joint state has not been published yet, return none
        if self.thetas is None:
            return None, None
        # Obtain end effector configuration
        T = PoE(self.Tsb, self.screws, self.thetas)
        R, p = TransToRp(T)
        pitch = np.pi/2 - np.sum(self.thetas[1:])
        return p, pitch

    def run(self):
        while not rospy.is_shutdown():
            while self.theta_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            self.theta_stale = True
            p, pitch = self.get_current_pos()
            self.coll_hand.update(p, pitch)
            print('LOOP')
            print(f'Avoiding belt: {self.coll_hand.avoids_belt()}')
            print(f'Avoiding fence: {self.coll_hand.avoids_fence()}')
            print(f'Avoiding base: {self.coll_hand.avoids_base()}')
            print(f'Avoiding blocks: {self.coll_hand.avoids_blocks()}')
            print(f'Valid gripper pos: {self.coll_hand.valid_gripper_pos()}')
            print(f'Avoiding all: {self.coll_hand.avoids_all_collisions()}')
            p, pitch = self.get_current_pos()
            print(f'Pos = {p}, Pitch = {pitch}')
            print()

def main():
    cdt = CollisionDetectTester()
    cdt.run()
    rospy.spin()

if __name__ == '__main__':
    main()
