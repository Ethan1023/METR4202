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
from constants import THETA_RANGES, ERROR_TOL, MAX_JOINT_VEL

class JointController:
    '''
    Publish to desired_joint_state
    Subscribe to joint_states
    Implement as action?
    '''
    def __init__(self):
        self.other_init()
        self.rospy_init()
   
    def rospy_init(self):
        # Create node
        rospy.init_node('joint_controller', anonymous=False)
        # Publish to desired joint states
        #self.joint_pub = rospy.Publisher('desired_joint_states_raw', JointState, queue_size=1)
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Subscribe to actual joint states
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)

        while len(self.joint_names) == 0:
            # Block operation until state vector obtained
            time.sleep(0.01)
    
    def other_init(self):
        self.stale = True
        # Variables for storing most recent joint state
        # Note that a list in ROS is interpreted as a tuple
        self.joint_names = ()
        self.joint_positions = ()
        self.joint_velocities = ()
        self.joint_efforts = ()
        # Joint names
        self.names = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        # Constants for forwards kinematics
        self.Tsb, self.screws = derivePoE()
        self.last_time = time.time()
        
        self.max_vel = np.array(MAX_JOINT_VEL)

        self.printing = True
        self.ERROR = False

    def joint_state_callback(self, joint_state):
        '''
        Keep actual joint state up to date
        '''
        self.joint_names = joint_state.name  # TODO - make this only be set once, or hard code?
        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_efforts = joint_state.effort
        self.last_time = time.time()
        self.stale = False

    def joint_state_publisher(self, desired_pos, desired_vel=None):
        '''
        Publish desired joint angles and velocities
        '''
        if self.ERROR:
            print(f'Error encountered, crashing')
            assert 1 == 0
        joint_state = JointState()
        joint_state.name = self.names
        joint_state.position = desired_pos
        print(f'desired_pos = {joint_state.position}')
        # Limit joint angles
        for i in range(len(desired_pos)):
            if desired_pos[i] > THETA_RANGES[i][1]:
                print(f'WARNING - Desired pos too high, saturating')
                desired_pos[i] = THETA_RANGES[i][1]
            elif desired_pos[i] < THETA_RANGES[i][0]:
                print(f'WARNING - Desired pos too high, saturating')
                desired_pos[i] = THETA_RANGES[i][0]
        if desired_vel is not None:
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

    def end_effector_publisher(self, desired_coords, desired_pitch, desired_vel=None):
        '''
        Publish joint angles and velocities required to reach end effector configuration
        '''
        possible = inv_kin(desired_coords, desired_pitch, check_possible=True)
        print(f'Desired coords = {desired_coords}')
        if possible:
            thetas = inv_kin(desired_coords, desired_pitch)
            print(f'Desired thetas = {thetas}')
            self.joint_state_publisher(thetas, desired_vel)
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
            target_coords = current_coords + frac * (desired_coords - current_coords)
            target_pitch = current_pitch + frac * (desired_pitch - current_pitch)
            #target_coords = desired_coords
            #target_pitch = desired_pitch
            # will be constant if target coords were reached in time
            target_coords_vel = (desired_coords - current_coords) / t
            target_pitch_vel = (desired_pitch - current_pitch) / t
            # calculate required joint velocities
            print(f'target_vel = {target_coords_vel}, {target_pitch_vel}')
            target_joint_vel = self.calc_joint_vel(self.thetas, target_coords_vel, target_pitch_vel)
            print(f'Target joint vel = {target_joint_vel}')
            #target_joint_vel = np.array([0, 0, 0, 0])
            #target_joint_vel = self.calc_joint_vel(inv_kin(current_coords, current_pitch), target_coords_vel, target_pitch_vel)
            #print(inv_kin(current_coords, current_pitch))
            # move to next position
            self.end_effector_publisher(target_coords, target_pitch, target_joint_vel)
            rate.sleep()
            # Update current coordinates
            current_coords, current_pitch = self.get_current_pos()
            #current_coords = target_coords  # TODO - remove
            #current_pitch = target_pitch  # TODO - remove
        self.end_effector_publisher(desired_coords, desired_pitch, [0, 0, 0, 0])

    def go_to_pos2(self, desired_coords, desired_pitch, coords_vel = 0.05, pitch_gain = 1):
        '''
        Runs at constant velocity
        '''
        # TODO - computational and experimental testing required
        current_coords = None
        current_pitch = None
        while current_coords is None:
            current_coords, current_pitch = self.get_current_pos()  # Updates self.thetas
        #current_coords, current_pitch = (np.array([0, 0, 0]), 0)  # TODO - remove
        coord_error = np.sum((current_coords - desired_coords)**2)**0.5
        pitch_error = abs(current_pitch-desired_pitch)
        error = (coord_error**2 + pitch_error**2)**0.5
        print(f'init error = {error} = {coord_error} + {pitch_error}')
        while error > ERROR_TOL and not rospy.is_shutdown():
            print(f'Current coords = {current_coords}, {current_pitch}')
            print(f'Current thetas = {self.thetas}')
            coords_diff = (np.abs(desired_coords - current_coords))**0.5
            coords_diff_mag = (np.sum(coords_diff**2))**0.5
            print(f'Coords error = {coords_diff}, {desired_pitch-current_pitch}')
            print(f'Coords mag = {coords_diff_mag}')
            print(f'Coords norm = {coords_diff / coords_diff_mag}')
            target_coords_vel = coords_diff / coords_diff_mag * coords_vel
            target_pitch_vel = (desired_pitch - current_pitch) * pitch_gain
            # calculate required joint velocities
            print(f'Target_vel = {target_coords_vel}, {target_pitch_vel}')
            target_joint_vel = self.calc_joint_vel(self.thetas, target_coords_vel, target_pitch_vel)
            print(f'Target joint vel = {target_joint_vel}')
            #target_joint_vel = np.array([0, 0, 0, 0])
            #target_joint_vel = self.calc_joint_vel(inv_kin(current_coords, current_pitch), target_coords_vel, target_pitch_vel)
            #print(inv_kin(current_coords, current_pitch))
            # move to next position
            self.end_effector_publisher(desired_coords, desired_pitch, target_joint_vel)
            # Wait for new values
            while self.stale and not rospy.is_shutdown:
                time.sleep(0.001)
            # Update current coordinates
            current_coords, current_pitch = self.get_current_pos()
            self.stale = True
            coord_error = np.sum((current_coords - desired_coords)**2)**0.5
            pitch_error = abs(current_pitch-desired_pitch) / 10
            error = (coord_error**2 + pitch_error**2)**0.5
            print(f'loop error = {error} = {coord_error} + {pitch_error}')
            print()
            print()

def main():
    # Create ROS node
    jc = JointController()
    # Prevent python from exiting
    test2(jc)
    rospy.spin()

def test2(jc):
    while not rospy.is_shutdown():
        jc.go_to_pos2(np.array([0.13, 0.05, 0.15]), -np.pi/2, 0.1, 2)
        time.sleep(1)
        jc.go_to_pos2(np.array([0.13, -0.05, 0.03]), -np.pi/2, 0.1, 2)
        time.sleep(1)

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
    while not rospy.is_shutdown():
        jc.go_to_pos2(np.array([0.13, 0, 0.15]), -np.pi/2, 0.05, 1)
        time.sleep(3)
        #jc.go_to_pos2(np.array([0.13, 0, 0.01]), -np.pi/2, 0.05, 1)
        #time.sleep(3)
        exit()
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
