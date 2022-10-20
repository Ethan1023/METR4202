#!/usr/bin/env python3

'''
TODO: documentation
'''

import math
import time

from threading import Lock

import numpy as np
import rospy

from std_msgs.msg import Bool, ColorRGBA, Float32
from sensor_msgs.msg import JointState
from metr4202.msg import BoxTransformArray, Pos, GripperState  # Custom messages from msg/

from inverse_kinematics import inv_kin
# from constants import EMPTY_HEIGHT, GRABBY_HEIGHT, CARRY_HEIGHT, ERROR_TOL, GRAB_TIME, \
#                       STATE_RESET, STATE_FIND, STATE_GRAB, STATE_COLOUR, STATE_PLACE, STATE_ERROR, STATE_TRAP, \
#                       STATE_TOSS, \
#                       L1, L2, L3, L4, PLACE_DICT, VELOCITY_AVG_TIME, OMEGA_THRESHOLD, BASE_TO_BELT, STATE_NAMES, \
#                       RAD_OFFSET, H_BLOCK
from maths import yaw_from_quat

from constants import *


class Box:
    def __init__(self, x, y, zrot, radius) -> None:
        self.x = None
        self.y = None
        self.zrot = None
        self.radius


class StateMachine:
    def __init__(self, dorospy: bool = True) -> None:
        self.other_init()
        if dorospy:
            self.rospy_init()
        print(f'StateMachine initialised')

    def other_init(self):
        self.box_lock = Lock()
        #self.theta_stale = True
        self.printing = True
        self.camera_stale = True
        self.position_error = ERROR_TOL*10
        # Assume joint controller runs faster than camera so no real need to track is position error is stale
        # TODO - camera input variables

        self.boxes = {}

        self.x_hist = []  # List of lists of position history
        self.y_hist = []
        self.t_hist = []  # List of lists of update times

        self.x_vels = []  # List of velocities
        self.y_vels = []
        
        self.omegas = []  # list of angular velocities
        self.omega = 0    # average angular velocity (0 if not known)
        self.start_time = time.time()   # update while any blocks are not moving
        self.stop_time = time.time()    # update while any blocks are moving
        self.old_stop_time = time.time()

        # variables to store current state
        self.state = STATE_RESET
        self.state_funcs = (self.state_reset, self.state_find, self.state_grab, \
                            self.state_colour, self.state_place, self.state_error,\
                            self.state_trap, self.state_toss)
        self.desired_id = None
        self.moving = True
        self.grab_moving = False

    def rospy_init(self):
        # Create node
        rospy.init_node('state_machine', anonymous=False)
        # Publish to desired end effector
        self.pos_pub = rospy.Publisher('desired_pos', Pos, queue_size=1)
        # Publish to request box colour
        self.colour_pub = rospy.Publisher('request_colour', Bool, queue_size=1)
        # Publish to desired joint states for direct access
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Publish to gripper
        self.gripper_pub = rospy.Publisher('gripper_state', GripperState, queue_size=1)
        # Subscribe to camera - TODO
        rospy.Subscriber('box_transforms', BoxTransformArray, self.camera_callback)
        # Subscribe to angle error - TODO
        rospy.Subscriber('position_error', Float32, self.position_error_callback)
        # Subscribe to colour detection
        rospy.Subscriber('box_colour', ColorRGBA, self.colour_detect_callback)

        while self.camera_stale and not rospy.is_shutdown():
            # Block operation until camera data received
            time.sleep(0.01)

    def camera_callback(self, box_transforms: BoxTransformArray) -> None:
        '''
        TODO: documentation
        '''
        # Acquire lock on boxes so they are not accessed while being updated
        self.box_lock.acquire()

        # Add new boxes or update existing ones from camera data
        for box_transform in box_transforms.transforms:
            box_id = box_transform.fiducial_id

            # Update existing boxes
            if box_id in self.blocks:

                box = self.boxes[box_id]
                box.x = box_transform.transform.translation.x
                box.y = box_transform.transform.translation.y
                box.zrot = yaw_from_quat(box_transform.transform.rotation)
                box.radius = np.sqrt((box.x - BASE_TO_BELT)**2 + box.y**2)

                self.x_hist[i].append(box.x)
                self.y_hist[i].append(box.y)
                self.t_hist[i].append(time.time())

            # Add new boxes
            else:
                print(f'New box found!')

                x = box_transform.transform.translation.x
                y = box_transform.transform.translation.y
                zrot = yaw_from_quat(box_transform.transform.rotation)
                radius = np.sqrt((x - BASE_TO_BELT)**2 + y**2)

                self.boxes[box_id] = Box(x, y, zrot, radius)

                # Create 'queue' with new position
                self.x_hist.append([x])
                self.y_hist.append([y])
                self.t_hist.append([time.time()])

                # Velocity is unknown
                self.x_vels.append(None)
                self.y_vels.append(None)
                self.omegas.append(None)

        # If oldest history >X seconds old, delete (FIFO queue)
        # Calculate velocity with oldest pos <X sec old and most recent val
        for i, (t_hist, x_hist, y_hist) in enumerate(zip(self.t_hist, self.x_hist, self.y_hist)):
            if len(t_hist) > 1:
                if t_hist[-1] - t_hist[0] > VELOCITY_AVG_TIME:  # if oldest value is old enough
                    # While second oldest value is old enough, clear oldest
                    while len(t_hist) > 2 and t_hist[-1] - t_hist[1] > VELOCITY_AVG_TIME:
                        del x_hist[0]
                        del y_hist[0]
                        del t_hist[0]
                    # Calculate velocities
                    print(f'{self.x_vels[i]}')
                    print(f'={self.xs[i]}')
                    print(f'- {t_hist}')
                    self.x_vels[i] = (self.xs[i] - x_hist[0]) / (t_hist[-1] - t_hist[0])
                    self.y_vels[i] = (self.ys[i] - y_hist[0]) / (t_hist[-1] - t_hist[0])
                    self.omegas[i] = np.sqrt(self.x_vels[i]**2 + self.y_vels[i]**2) / self.radii[i]
            else:
                # Set to none if can't track? TODO
                self.x_vels[i] = None
                self.y_vels[i] = None
                self.omegas[i] = None
        total = 0
        count = 0
        for omega in self.omegas:
            if omega is not None:
                total += omega
                count += 1
        self.box_lock.release()
        if not count == 0:
            self.omega = total / count
        else:
            self.omega = 0
        # Update timers
        #maxvel = -1
        #for x_vel, y_vel in zip(self.x_vels, self.y_vels):
        #    if x_vel is not None:
        #        maxvel = max(maxvel, np.sqrt(x_vel**2 + y_vel**2))
        #if maxvel > VELOCITY_THRESHOLD:
        if self.omega > OMEGA_THRESHOLD:
            self.old_stop_time = self.start_time
            self.stop_time = time.time()
            self.moving = True
            print(f'Time since started = {self.stop_time - self.start_time} s')
        else:
            self.start_time = time.time()
            self.moving = False
            print(f'Time since stopped = {self.start_time - self.stop_time} s')
        if len(self.boxes) > 0:
            self.camera_stale = False

    def colour_detect_callback(self, data: ColorRGBA) -> None:
        '''
        Updates the self.detected_colour variable with the received response.
        '''
        print(f'colour callback')
        self.detected_colour = data

    def request_colour(self) -> ColorRGBA:
        '''
        Publishes a request to the "colour_request" topic and subscribes to the
        "box_colour" topic to receive the response. Returns ColorRGBA.
        '''
        self.detected_colour = None
        request = Bool(); request.data = True
        self.colour_pub.publish(request)

        while not self.detected_colour and not rospy.is_shutdown():
            time.sleep(0.01)
        return self.detected_colour

    def position_error_callback(self, msg):
        # TODO - track change in position error to see if robot is still moving
        # print(f'pos error callback = {msg.data}')
        self.position_error = msg.data

    def desired_pos_publisher(self, coords, pitch=None, rad_offset=0):
        '''
        Accept coords and optional pitch
        Return if command was issued
        if pitch is not supplied, try as straight down as possible
        '''
        self.position_error = ERROR_TOL*10
        pos = Pos()
        pos.x, pos.y, pos.z = coords
        if not rad_offset == 0:
            rad_orig = np.sqrt(pos.x**2 + pos.y**2)
            rad_new = rad_orig + rad_offset
            pos.x = pos.x * rad_new / rad_orig
            pos.y = pos.y * rad_new / rad_orig
        if pitch is not None:  # if pitch supplied, try to reach it or fail
            if inv_kin(coords, pitch, self_check=False, check_possible=True):
                pos.pitch = pitch
                self.pos_pub.publish(pos)
                return True
            print(f'NOT POSSIBLE')
            return False
        else:  # Otherwise use default and increment
            for pitch in np.linspace(-np.pi/2, 0, 10):
                if inv_kin(coords, pitch, self_check=False, check_possible=True):
                    pos.pitch = pitch
                    self.pos_pub.publish(pos)
                    return True
            print(f'NOT POSSIBLE')
            return False

    def command_gripper(self, open_gripper: bool = True) -> None:
        '''
        Publishes a command to the open or grip the gripper. Blocks for a
        period defined by "GRAB_TIME".
        '''
        msg = GripperState(); msg.open = open_gripper
        self.gripper_pub.publish(msg)
        time.sleep(GRAB_TIME)

    def delete_box(self, box_id):
        self.box_lock.acquire()
        i = self.ids.index(box_id)
        del self.boxes[box_id]
        self.box_lock.release()

    def run(self):
        while not rospy.is_shutdown():
            while self.camera_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            #self.camera_stale = True
            self.loop()

    def moving_grab_update(self):
        if not self.moving and (time.time()-self.stop_time) > 2: #TODO - change to a constant
            self.grab_moving = False
        elif self.moving and (self.start_time - self.old_stop_time) > 2:
            self.grab_moving = False
        else:
            self.grab_moving = True
        print(f"Belt moving = {self.grab_moving}")
    
    def grab_closest(self):
        '''
        returns: id of block with smallest radius (block to be grabbed first)
        '''
        closest_id = None; min_distance = math.inf

        for box_id in self.boxes:
            box = self.boxes[box_id]
            dist = np.hypot(box.x, box.y)
            if dist < min_distance:
                closest_id = box_id
                min_distance = dist

        return closest_id

    def pickup_block(self, block_id):
        i = self.ids.index(block_id)
        x = self.xs[i]
        y = self.ys[i]
        z = EMPTY_HEIGHT
        coords = (x, y, z)
        possible = self.desired_pos_publisher(coords, rad_offset=RAD_OFFSET)
        if not possible:
            return STATE_FIND
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        z = GRABBY_HEIGHT
        coords = (x, y, z)
        possible = self.desired_pos_publisher(coords, rad_offset=RAD_OFFSET)
        if not possible:
            return STATE_RESET
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        time.sleep(0.2)
        self.command_gripper(open_gripper=False)
        time.sleep(GRAB_TIME)
        z = CARRY_HEIGHT
        coords = (x, y, z)
        possible = self.desired_pos_publisher(coords, rad_offset=RAD_OFFSET)
        if not possible:
            return STATE_RESET
        return None

    def loop(self):
        '''
        State machine main loop
        read current state
        use block position and/or position error to change state or issue command to gripper or joint_controller
        '''
        # do logic
        # if len(self.ids) == 1:
        #     state_1(self.xs[0], self.ys[0])
        print(f"State = {STATE_NAMES[self.state]}")
        self.state = self.state_funcs[self.state]()
        # self.pickup_block(0)
        # publsh commands if needed

    def move_to(self, coords, pitch: float, rad_offset: int = 0) -> bool:
        '''
        A wrapper around desired_pos_publisher that handles waiting for the
        end effector to reach the desired position.
        '''
        self.desired_pos_publisher(coords, pitch, rad_offset)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        time.sleep(0.1)

    def state_reset(self):
        '''Moves the robot into the idle position with the gripper open.'''
        self.command_gripper(open_gripper=True)
        self.move_to(POSITION_IDLE, 0)
        return STATE_FIND

    def state_find(self):
        # Locating and choosing which block to pick up
        # TODO: ADD LOGIC
        while len(self.ids) == 0 and not rospy.is_shutdown():
            time.sleep(0.01)
        #self.moving_grab_update()
        self.desired_id = self.grab_closest()
        return STATE_GRAB

    def state_grab(self):
        # Picking up a block
        # If fails, open gripper and return to state_find
        if self.moving:
            return STATE_GRAB
        alt_state = self.pickup_block(self.desired_id)
        if alt_state is not None:
            return alt_state
        return STATE_COLOUR

    def state_colour(self):
        '''
        TODO: documentation
        '''
        # Checking the block colour
        # If fails, open gripper and return to state_find
        coords = (BASE_TO_BELT, 0, COLOUR_DETECT_HEIGHT)
        pitch = 0
        self.desired_pos_publisher(coords, pitch)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        # print(f'Requestself.desired_idlour')
        # self.request_colour()
        # print(f'Colour = {self.detected_colour}')
        return STATE_PLACE

    def state_place(self) -> None:
        '''
        Move the end effector from the colour detection pose to the drop-off
        zone corresponding to the desired colour.
        '''
        # Get the x, y coords of the desired drop-off zone
        # dropoff_position = DROPOFF_POSITION[self.detected_colour]
        coords = DROPOFF_POSITION['red'] # TODO: un-hardcode this

        # To avoid collision, from the colour detection pose first rotate the
        # base to the x, y coordinate of the desired drop-off zone
        coords = (-0.1, 0.1, COLOUR_DETECT_HEIGHT) # TODO: un-hardcode the x, y
        self.move_to(coords, pitch=0)

        # Move end-effector directly down to place the block
        coords = (coords[0], coords[1], DROPOFF_HEIGHT)
        self.move_to(coords, pitch=-np.pi/2)

        # Release the block
        self.command_gripper(open_gripper=True)

        # Move the arm back up so it can return to reset position
        coords = (coords[0], coords[1], CARRY_HEIGHT)
        self.move_to(coords, pitch=-np.pi/2)

        # Return to reset state
        return STATE_RESET

    def state_toss(self):
        coords = (L4, 0, L1+L2+L3)
        self.desired_pos_publisher(coords, 0)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)

        joint_state = JointState()
        joint_state.name = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        joint_state.position = (0, 0, 0, 0)
        joint_state.velocity = (5, 5, 5, 10)
        self.joint_pub.publish(joint_state)
        time.sleep(0.3)
        self.command_gripper(open_gripper=True)
        time.sleep(0.5)
        joint_state = JointState()
        joint_state.name = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        joint_state.position = (0, 0, 0, np.pi/2)
        joint_state.velocity = (5, 5, 5, 10)
        self.joint_pub.publish(joint_state)

        time.sleep(1)
        self.delete_box(self.desired_id)
        return STATE_RESET

    def state_error(self):
        # If called, open gripper and go to state_reset
        self.command_gripper(open_gripper=True)
        return STATE_RESET

    def state_trap(self):
        return STATE_TRAP


if __name__ == '__main__':
    # Create ROS node
    sm = StateMachine()
    # Prevent python from exiting
    sm.run()
    rospy.spin()
