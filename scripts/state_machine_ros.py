#!/usr/bin/env python3

'''
TODO: documentation
'''

import math
import time

from threading import Lock

import numpy as np
import rospy
from collections import Counter

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState
from metr4202.msg import BoxTransformArray, Pos, GripperState, Thetas  # Custom messages from msg/

from inverse_kinematics import inv_kin
# from constants import EMPTY_HEIGHT, GRABBY_HEIGHT, CARRY_HEIGHT, ERROR_TOL, GRAB_TIME, \
#                       STATE_RESET, STATE_FIND, STATE_GRAB, STATE_COLOUR, STATE_PLACE, STATE_ERROR, STATE_TRAP, \
#                       STATE_TOSS, \
#                       L1, L2, L3, L4, PLACE_DICT, VELOCITY_AVG_TIME, OMEGA_THRESHOLD, BASE_TO_BELT, STATE_NAMES, \
#                       RAD_OFFSET, H_BLOCK, COLOUR_CHECK_TIME, MAX_BLOCK_AGE
from maths import yaw_from_quat

from constants import *


class Box:
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.zrot = None
        self.radius = None

        self.x_hist = []
        self.y_hist = []
        self.t_hist = []

        self.angular_velocity = None

    def update(self, x, y, zrot, timestamp) -> None:
        '''
        Updates the current values and history of this box with the new x, y.
        '''
        self.x = x
        self.y = y
        self.zrot = zrot
        self.radius = np.sqrt((x - BASE_TO_BELT)**2 + y**2)

        self.x_hist.append(x)
        self.y_hist.append(y)
        self.t_hist.append(timestamp)

        self.update_apparent_velocity()

    def update_apparent_velocity(self) -> None:
        '''Updates the apparent velocity using the historical values.'''
        # If we don't have sufficient data, return without calculating
        if self.t_hist[-1] - self.t_hist[0] < VELOCITY_AVG_TIME:
            self.angular_velocity = None
            return

        # Remove unnecessarily old values from history
        while len(self.t_hist) > 2 and self.t_hist[-1] - self.t_hist[1] > VELOCITY_AVG_TIME:
            del self.x_hist[0]
            del self.y_hist[0]
            del self.t_hist[0]

        # Calculate velocity
        v_x = (self.x_hist[-1] - self.x_hist[0]) / (self.t_hist[-1] - self.t_hist[0])
        v_y = (self.y_hist[-1] - self.y_hist[0]) / (self.t_hist[-1] - self.t_hist[0])

        self.angular_velocity = np.linalg.norm([v_x, v_y]) / self.radius


class StateMachine:
    def __init__(self, dorospy: bool = True) -> None:
        self.other_init()
        if dorospy:
            self.rospy_init()
        print(f'StateMachine initialised')
        rospy.loginfo(f'StateMachine initialised')

    def other_init(self):
        self.box_lock = Lock()
        #self.theta_stale = True
        self.printing = True
        self.camera_stale = True
        self.position_error = ERROR_TOL*10
        # NOTE: Assume joint controller runs faster than camera so no real
        # need to track is position error is stale

        self.boxes = {}

        self.x_vels = []  # List of velocities
        self.y_vels = []
        
        self.omegas = []  # list of angular velocities
        self.omega = 0    # average angular velocity (0 if not known)
        self.last_stopped_time = time.time()   # update while any blocks are not moving
        self.last_moved_time = time.time()    # update while any blocks are moving
        self.old_stop_time = time.time()

        self.colour_check_time = time.time()
        self.detected_colour = None

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
        # Publish to desired joint states for direct access
        self.theta_pub = rospy.Publisher('desired_thetas', Thetas, queue_size=1)
        # Publish to gripper
        self.gripper_pub = rospy.Publisher('gripper_state', GripperState, queue_size=1)
        # Subscribe to camera - TODO
        rospy.Subscriber('box_transforms', BoxTransformArray, self.camera_callback)
        # Subscribe to angle error - TODO
        rospy.Subscriber('position_error', Float32, self.position_error_callback)
        # Subscribe to colour detection
        rospy.Subscriber('box_colour', String, self.colour_detect_callback)
        
        # Reset before camera data
        self.state_reset()

        while self.camera_stale and not rospy.is_shutdown():
            # Block operation until camera data received
            time.sleep(0.01)

    def camera_callback(self, box_transforms: BoxTransformArray) -> None:
        '''
        TODO: documentation
        '''
        # Acquire lock on boxes so they are not accessed while being updated
        self.box_lock.acquire()

        # Define time of camera data
        timestamp = time.time()

        # Add new boxes or update existing ones from camera data
        for box_transform in box_transforms.transforms:
            box_id = box_transform.fiducial_id

            # Update existing boxes
            if box_id in self.boxes:

                box = self.boxes[box_id]

                x = box_transform.transform.translation.x
                y = box_transform.transform.translation.y
                zrot = yaw_from_quat(box_transform.transform.rotation)

                box.update(x, y, zrot, timestamp)

            # Add new boxes
            else:
                x = box_transform.transform.translation.x
                y = box_transform.transform.translation.y
                zrot = yaw_from_quat(box_transform.transform.rotation)

                new_box = Box()
                new_box.update(x, y, zrot, timestamp)
                self.boxes[box_id] = new_box

        # Average velocity of all boxes for which velocity ahs been calculated
        v_sum = 0; v_count = 0
        for box in self.boxes.values():
            if box.angular_velocity:
                v_sum += box.angular_velocity
                v_count += 1
        self.omega = v_sum / v_count if v_count else None

        # Release lock on boxes
        self.box_lock.release()

        # Update instance variables tracking belt start and stop times
        if self.omega:
            if self.omega > OMEGA_THRESHOLD:
                self.last_moved_time = time.time()
                self.moving = True
                rospy.logdebug(f'Time since started = {self.last_moved_time - self.last_stopped_time} s')
            else:
                self.last_stopped_time = time.time()
                self.moving = False
                rospy.logdebug(f'Time since stopped = {self.last_stopped_time - self.last_moved_time} s')

        # Update "stale" status of camera
        if len(self.boxes) > 0:
            self.camera_stale = False

    def colour_detect_callback(self, colour: String) -> None:
        '''
        Updates the self.detected_colour variable with the received response.
        '''
        self.detected_colour = colour.data

    def request_colour(self) -> String:
        '''
        Publishes a request to the "colour_request" topic and subscribes to the
        "box_colour" topic to receive the response. Returns ColorRGBA.
        '''
        self.colour_check_time = time.time()
        rospy.loginfo('request_colour: starting')
        request = Bool(); request.data = True
        detected_colours = []
        while self.detected_colour is None or (len(detected_colours)<COLOUR_CHECK_SAMPLES and time.time() - self.colour_check_time < COLOUR_CHECK_TIME):
            rospy.loginfo('request_colour: requesting colour')
            self.detected_colour = None
            self.colour_pub.publish(request)
            while not self.detected_colour and not rospy.is_shutdown():
                time.sleep(0.01)
            if not self.detected_colour == 'other':
                detected_colours.append(self.detected_colour)
        if len(detected_colours) == 0:
            return 'other'
        counter = Counter(detected_colours)
        colour = counter.most_common(n=1)[0][0]
        rospy.loginfo(f'request_colour: returning {colour}')
        self.detected_colour = colour
        return colour

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
        if box_id in self.boxes:
            del self.boxes[box_id]
        self.box_lock.release()

    def delete_old_boxes(self):
        self.box_lock.acquire()
        curr_time = time.time()
        dictkeys = list(self.boxes.keys())
        for box_id in dictkeys:
            if curr_time - self.boxes[box_id].t_hist[-1] > MAX_BLOCK_AGE:
                rospy.loginfo(f'delete_old_boxes: deleting {box_id}')
                del self.boxes[box_id]
        self.box_lock.release()
        

    def run(self):
        while not rospy.is_shutdown():
            while self.camera_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            #self.camera_stale = True
            rospy.loginfo('run: looping')
            self.delete_old_boxes()
            self.loop()

    # def grab_moving(self):
    #     '''
    #     Decides whether we should pick up a box while the belt is moving or if
    #     we should wait for it to stop. Returns True to pick up while moving.
    #     '''
    #     # If the belt is currently stopped, we can decide to not grab moving
    #     # if the belt stopped more than X seconds ago
    #     if not self.moving:
    #         if time.time() - self.last_moved_time > TASK3B_THRESHOLD:
    #             self.grab_moving = False

    #     # If the belt is currently moving, we can decide to grab moving if the
    #     # last period of stopping was less than X seconds
    #     else:
    #         if self.last_stopping_duration < TASK3B_THRESHOLD:
    #             self.grab_moving = True
    
    def grab_closest(self):
        '''
        returns: id of block with smallest radius (block to be grabbed first)
        '''
        closest_id = None; min_distance = math.inf

        for box_id in self.boxes:
            box = self.boxes[box_id]
            dist = np.sqrt(box.x**2 + box.y**2)
            if dist < min_distance:
                closest_id = box_id
                min_distance = dist

        return closest_id

    def pickup_block(self, box_id: str) -> int:
        '''
        Commands joints and end effector to pick up the box with the given ID.
        If 
        '''
        # Get the current coordinates of the desired box
        box = self.boxes[box_id]; x = box.x; y = box.y; z = EMPTY_HEIGHT
         
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
            return None
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        return None

    def loop(self):
        '''
        State machine main loop
        read current state
        use block position and/or position error to change state or issue command to gripper or joint_controller
        '''
        rospy.loginfo(f"State = {STATE_NAMES[self.state]}")
        self.state = self.state_funcs[self.state]()

    def move_to(self, coords, pitch: float, rad_offset: int = 0) -> bool:
        '''
        A wrapper around desired_pos_publisher that handles waiting for the
        end effector to reach the desired position.
        '''
        self.desired_pos_publisher(coords, pitch, rad_offset)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)

    def state_reset(self):
        '''Moves the robot into the idle position with the gripper open.'''
        self.command_gripper(open_gripper=True)
        # self.move_to(POSITION_IDLE, pitch=0)
        #joint_state = JointState()
        #joint_state.name = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        #joint_state.position = (0, 0, 0, np.pi/2)
        #joint_state.velocity = (3, 3, 3, 6)
        #self.joint_pub.publish(joint_state)
        #time.sleep(0.5)
        thetas = Thetas()
        thetas.thetas = (0, 0, 0, np.pi/2)
        self.position_error = ERROR_TOL*10
        self.theta_pub.publish(thetas)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        return STATE_FIND

    def state_find(self):
        '''
        Sets self.desired_id to the block which should be picked first.
        '''
        rospy.loginfo(f'state_find: starting')
        while len(self.boxes) == 0 and not rospy.is_shutdown():
            time.sleep(1)
            return STATE_FIND
            #time.sleep(0.01)

        rospy.loginfo(f'state_find: boxes exist')
        # TODO
        #while self.moving and self.grab_moving == False and not rospy.is_shutdown():
        #    time.sleep(0.01)

        rospy.loginfo(f'state_find: selecting box')
        self.desired_id = self.grab_closest()
        # # Check that angle of desired block is less than 30 degrees, 
        # # if more than 30 degrees, wait until stopped again
        # # zrot is constrained to being between 0 and 45 degrees
        # if abs(self.boxes[self.desired_id].zrot)/4 > ZROT_LIMIT:
        #     while self.moving == False:
        #         time.sleep(0.01)
        #         # if it's stopped for more than 10s, task 3a
        #         if (self.last_stopped_time - self.last_moved_time) > 10:
        #             #TODO - 3a logic
        #             return STATE_GRAB
        #         # otherwise assume task 1 or 2 and wait for better orientation
        #         return STATE_FIND
        rospy.loginfo(f'state_find: returning')
        return STATE_GRAB

    def state_grab(self):
        '''
        Pick up the block with the ID chosen during the previous state.
        '''
        rospy.loginfo(f'state_grab: starting')
        alt_state = self.pickup_block(self.desired_id)
        if alt_state is not None:
            return alt_state
        rospy.loginfo(f'state_grab: got block')

        # Move the block to an intermediate position closer to the base to
        # avoid hitting other block when moving towards the 
        # self.move_to(POSITION_INTERMEDIATE, pitch=-np.pi/2)

        # Advance to the next state

        rospy.loginfo(f'state_grab: returning')
        return STATE_COLOUR

    def state_colour(self):
        '''
        TODO: documentation
        '''
        # Checking the block colour
        # If fails, open gripper and return to state_find
        coords = POSITION_COLOUR_DETECT
        self.move_to(coords, pitch=np.pi/6)

        detected_colour = self.request_colour()

        if detected_colour == 'other':
            return STATE_RESET

        return STATE_PLACE

    def state_place(self) -> None:
        '''
        Move the end effector from the colour detection pose to the drop-off
        zone corresponding to the desired colour.
        '''
        # Get the x, y coords of the desired drop-off zone
        coords = DROPOFF_POSITION[self.detected_colour]
        # coords = DROPOFF_POSITION['green'] # TODO: un-hardcode this

        # To avoid collision, from the colour detection pose first rotate the
        # base to the x, y coordinate of the desired drop-off zone
        p = np.array(coords)
        x, y = np.linalg.norm([0.1, 0.1]) / np.linalg.norm(p) * p
        coords = (x, y, COLOUR_DETECT_HEIGHT)
        # coords = (-0.1, 0.1, COLOUR_DETECT_HEIGHT)
        self.move_to(coords, pitch=0)

        # Move end-effector directly down to place the block
        coords = (coords[0], coords[1], DROPOFF_HEIGHT)
        self.move_to(coords, pitch=-np.pi/2)

        # Release the block
        time.sleep(0.3)
        self.command_gripper(open_gripper=True)

        # # Move the arm back up slightly before returning to reset position
        # coords = (coords[0], coords[1], CARRY_HEIGHT)
        # self.move_to(coords, pitch=-np.pi/2)

        # Delete the box from tracked boxes
        self.delete_box(self.desired_id)

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
