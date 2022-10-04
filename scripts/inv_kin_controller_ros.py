#!/usr/bin/env python3

'''
Use this - https://github.com/UQ-METR4202/dynamixel_interface/blob/master/tutorials/tutorial_1_using_the_controller.md
'''
import rospy
import numpy as np
import time
from modern_robotics import TransToRp
from sensor_msgs.msg import JointState
from inverse_kinematics import inv_kin, atan2
from forward_kinematics import derivePoE, PoE, derive_inv_jac, calc_frame1_vel
from metr4202.msg import Pos  # Custom messages from msg/

class JointController:
    '''
    Publish to desired_joint_state
    Subscribe to joint_states
    Implement as action?
    '''
    def __init__(self):
        # Create node
        rospy.init_node('inv_kin_controller', anonymous=False)
        # Publish to desired joint states
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Subscribe to actual joint states
        rospy.Subscriber('desired_pos', Pos, self.end_pos_callback)
        # Joint names
        self.names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        # Constants for forwards kinematics
        self.Tsb, self.screws = derivePoE()
        
        self.ERROR = False
        self.printing = True

    def end_pos_callback(self, pos):
        coords = np.array([pos.x, pos.y, pos.z])
        possible = self.end_effector_publisher(coords, pos.pitch)
        if not possible:
            print(f'ERROR: position not possible')

    def joint_state_publisher(self, desired_pos):
        '''
        Publish desired joint angles and velocities
        '''
        if self.ERROR:
            print(f'Error encountered, crashing')
            assert 1 == 0
        joint_state = JointState()
        joint_state.name = self.names
        joint_state.position = desired_pos
        joint_state.velocity = (0.5, 0.5, 0.5, 0.5)
        print(f'desired_pos = {joint_state.position}')
        self.joint_pub.publish(joint_state)

    def get_current_pos(self):
        '''
        Get current end effector position and pitch
        '''
        # If joint state has not been published yet, return none
        if len(self.joint_names) == 0:
            return None, None
        # Rearrange thetas in order from 1 to 4
        self.thetas = []
        for name in self.names:
            for theta, new_name in zip(self.joint_positions, self.joint_names):
                if name == new_name:
                    self.thetas.append(theta)
        # Obtain end effector configuration
        T = PoE(self.Tsb, self.screws, self.thetas)
        R, p = TransToRp(T)
        pitch = np.pi/2 - np.sum(self.thetas[1:])
        return p, pitch

    def end_effector_publisher(self, desired_coords, desired_pitch):
        '''
        Publish joint angles and velocities required to reach end effector configuration
        '''
        possible = inv_kin(desired_coords, desired_pitch, check_possible=True)
        print(f'Desired coords = {desired_coords}')
        if possible:
            thetas = inv_kin(desired_coords, desired_pitch)
            print(f'Desired thetas = {thetas}')
            self.joint_state_publisher(thetas)
        return possible

    def calc_joint_vel(self, thetas, coords_vel, pitch_vel):
        '''
        Use jacobian taken at point L1 to calculate joint velocities from current joint position and end effector velocity
        It is taken at L1 as a this point omega x and y velocities and jacobian rows are always zero making it possible to solve for joint velocities by simply
        removing the zero rows of the jacobian and inverting it
        '''
        # Oh god these funtions were painful to create
        J1inv = derive_inv_jac(thetas, printing=False)  # Find inverse 4DOF Jacobian at point L1 (after first joint)
        # Find 4DOF velocity at this point
        L1_4DOF_vel = calc_frame1_vel(thetas, coords_vel, pitch_vel, printing=False, Tsb=self.Tsb, screws=self.screws, ignore=True)
        # Calculate joint velocities
        thetas_vel = np.matmul(J1inv, L1_4DOF_vel)
        return thetas_vel

    def go_to_pos(self, desired_coords, desired_pitch, time=1, steps=100):
        '''
        Adjusts velocity to reach desired in target time (i.e. will speed up if obstructed)
        Will likely overshoot a bit?
        '''
        # TODO - computational and experimental testing required
        current_coords = None
        current_pitch = None
        while current_coords is None:
            current_coords, current_pitch = self.get_current_pos()  # Updates self.thetas
        #current_coords, current_pitch = (np.array([0, 0, 0]), 0)  # TODO - remove
        rate = rospy.Rate(steps/time)
        for t, frac in zip(np.linspace(time, time/steps, steps), 1/np.linspace(steps, 1, steps)):
            print('loop')
            print(f'Current coords = {current_coords}, {current_pitch}')
            print(f'Current thetas = {self.thetas}')
            # t is total remaining time
            # frac is how far to solution we need to reach before the next loop
            # if velocity is constant, these will be equally spaced
            #target_coords = current_coords + frac * (desired_coords - current_coords)
            #target_pitch = current_pitch + frac * (desired_pitch - current_pitch)
            target_coords = desired_coords
            target_pitch = desired_pitch
            # will be constant if target coords were reached in time
            target_coords_vel = (desired_coords - current_coords) / t
            target_pitch_vel = (desired_pitch - current_pitch) / t
            # calculate required joint velocities
            print(f'target_vel = {target_coords_vel}, {target_pitch_vel}')
            target_joint_vel = self.calc_joint_vel(self.thetas, target_coords_vel, target_pitch_vel)
            #target_joint_vel = self.calc_joint_vel(inv_kin(current_coords, current_pitch), target_coords_vel, target_pitch_vel)
            #print(inv_kin(current_coords, current_pitch))
            print(f'Target joint vel = {target_joint_vel}')
            # move to next position
            self.end_effector_publisher(target_coords, target_pitch, target_joint_vel)
            rate.sleep()
            # Update current coordinates
            current_coords, current_pitch = self.get_current_pos()
            #current_coords = target_coords  # TODO - remove
            #current_pitch = target_pitch  # TODO - remove
            

def main():
    # Create ROS node
    jc = JointController()
    # Prevent python from exiting
#    test(jc)
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
    # Start at 0.13, 0, 0.15, -np.pi/2
    jc.go_to_pos(np.array([0.13, 0, 0.01]), -np.pi/2, 2, 400)
    exit()
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