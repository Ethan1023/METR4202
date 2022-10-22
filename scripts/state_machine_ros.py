#!/usr/bin/env python3

'''
TODO: documentation
'''

import math
import time

from threading import Lock  # Prevent weird behaviour when deleting boxes

import numpy as np
import rospy
from collections import Counter  # Obtain mode of colours seen

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState
from metr4202.msg import BoxTransformArray, Pos, GripperState, Thetas  # Custom messages from msg/

from inverse_kinematics import inv_kin
# from constants import EMPTY_HEIGHT, GRABBY_HEIGHT, CARRY_HEIGHT, ERROR_TOL, GRAB_TIME, \
#                       STATE_RESET, STATE_FIND, STATE_GRAB, STATE_COLOUR, STATE_PLACE, STATE_ERROR, STATE_TRAP, \
#                       STATE_TOSS, \
#                       L1, L2, L3, L4, PLACE_DICT, VELOCITY_AVG_TIME, OMEGA_THRESHOLD, BASE_TO_BELT, STATE_NAMES, \
#                       RAD_OFFSET, H_BLOCK, COLOUR_CHECK_TIME, MAX_BLOCK_AGE
from maths import yaw_from_quat  # Yaw angle of block

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
        rospy.loginfo('StateMachine other init completed')
        if dorospy:
            self.rospy_init()
        rospy.loginfo('StateMachine rospy init completed')
        self.state_reset()
        print(f'StateMachine initialised')
        rospy.loginfo(f'StateMachine initialised')

    def other_init(self):
        self.box_lock = Lock()
        #self.theta_stale = True
        #self.printing = True
        self.camera_stale = True  # Track is new camera info has been received
        self.position_error = ERROR_TOL*10  # Initialise position error

        self.boxes = {}  # Dictionary of boxes currently on belt

        #self.x_vels = []  # List of velocities
        #self.y_vels = []
        
        #self.omegas = []  # list of angular velocities
        self.omega = 0    # average angular velocity (0 if not known)
        self.last_stopped_time = time.time()   # update while any blocks are not moving
        self.last_moved_time = time.time()    # update while any blocks are moving
        self.old_stop_time = time.time()      # store previous stopped time

        self.colour_check_time = time.time()  # store when colour check began
        self.detected_colour = None           # store detected colour

        # Variables to store current state
        self.state = STATE_RESET
        # State functions
        self.state_funcs = (self.state_reset, self.state_find, self.state_grab, \
                            self.state_colour, self.state_place, self.state_error,\
                            self.state_trap, self.state_toss)
        self.desired_id = None    # id of desired box
        self.moving = True        # Track if belt is moving
        self.grab_moving = False  # Track if we want to grab moving

    def rospy_init(self):
        # Create node
        rospy.init_node('state_machine', anonymous=False)
        # Publish to desired end effector
        self.pos_pub = rospy.Publisher('desired_pos', Pos, queue_size=1)
        # Publish to request box colour
        self.colour_pub = rospy.Publisher('request_colour', Bool, queue_size=1)
        # Publish to desired joint states for direct access
        self.joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=1)
        # Publish to desired joint angles 
        self.theta_pub = rospy.Publisher('desired_thetas', Thetas, queue_size=1)
        # Publish to gripper
        self.gripper_pub = rospy.Publisher('gripper_state', GripperState, queue_size=1)
        rospy.loginfo('StateMachine publishers initialised')
        # Subscribe to camera
        rospy.Subscriber('box_transforms', BoxTransformArray, self.camera_callback)
        # Subscribe to angle error
        rospy.Subscriber('position_error', Float32, self.position_error_callback)
        # Subscribe to colour detection
        rospy.Subscriber('box_colour', String, self.colour_detect_callback)
        rospy.loginfo('StateMachine subscribers initialised')
        
        # Reset immediately

        #while self.camera_stale and not rospy.is_shutdown():
        #    # Block operation until camera data received
        #    time.sleep(0.01)

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
            print(f'x,y,z = {box_transform.transform.translation.x},{box_transform.transform.translation.y},{box_transform.transform.translation.z}')

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
        self.colour_check_time = time.time()  # Time colour request began
        rospy.loginfo('request_colour: starting')
        request = Bool(); request.data = True  # Prepare request
        detected_colours = []  # Store detected colours
        # Loop until required number of samples required or until timeout
        while len(detected_colours)<COLOUR_CHECK_SAMPLES and time.time() - self.colour_check_time < COLOUR_CHECK_TIME:
            # rospy.loginfo('request_colour: requesting colour')
            self.detected_colour = None
            self.colour_pub.publish(request)
            while not self.detected_colour and not rospy.is_shutdown():
                time.sleep(0.01)
            # It not other, store colour
            if not self.detected_colour == 'other':
                detected_colours.append(self.detected_colour)
        print(detected_colours)
        # If no colours found, return other
        if len(detected_colours) == 0:
            return 'other'
        # Find most common colour
        counter = Counter(detected_colours)
        colour = counter.most_common(n=1)[0][0]
        rospy.loginfo(f'request_colour: returning {colour}')
        self.detected_colour = colour
        return colour

    def position_error_callback(self, msg):
        # Save error from desired thetas
        self.position_error = msg.data

    def desired_pos_publisher(self, coords, pitch=None, rad_offset=0, dry=False):
        '''
        Accept coords and optional pitch
        Return if command was issued
        if pitch is not supplied, try as straight down as possible

        Parameters:
        coord - (x, y, z)
        pitch - radians
        rad_offset - radius offset
        dry - dry run (don't do anything, only return if possible)
        '''
        self.position_error = ERROR_TOL*10
        pos = Pos()
        pos.x, pos.y, pos.z = coords
        # Offset radius
        if not rad_offset == 0:
            rad_orig = np.sqrt(pos.x**2 + pos.y**2)
            rad_new = rad_orig + rad_offset
            pos.x = pos.x * rad_new / rad_orig
            pos.y = pos.y * rad_new / rad_orig
        if pitch is not None:  # if pitch supplied, try to reach it or fail
            if inv_kin(coords, pitch, self_check=False, check_possible=True):
                pos.pitch = pitch
                if not dry:
                    self.pos_pub.publish(pos)
                return True
            print(f'NOT POSSIBLE')
            return False
        else:  # Otherwise use default and increment
            for pitch in np.linspace(-np.pi/2, 0, 10):
                if inv_kin(coords, pitch, self_check=False, check_possible=True):
                    pos.pitch = pitch
                    if not dry:
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

    def delete_box(self, box_id):
        self.box_lock.acquire()
        if box_id in self.boxes:
            del self.boxes[box_id]
        self.box_lock.release()

    def delete_old_boxes(self):
        '''
        Delete boxes that haven't been seen in MAX_BLOCK_AGE seconds
        '''
        self.box_lock.acquire()
        curr_time = time.time()
        dictkeys = list(self.boxes.keys())
        for box_id in dictkeys:
            if curr_time - self.boxes[box_id].t_hist[-1] > MAX_BLOCK_AGE:
                rospy.loginfo(f'delete_old_boxes: deleting {box_id}')
                del self.boxes[box_id]
        self.box_lock.release()
        

    def run(self):
        '''
        Main loop
        Call loop until rospy exits
        '''
        while not rospy.is_shutdown():
            while self.camera_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            #self.camera_stale = True
            rospy.loginfo('run: looping')
            #time.sleep(5)
            self.loop()

    def grab_moving(self):
        '''
        Decides whether we should pick up a box while the belt is moving or if
        we should wait for it to stop. Returns True to pick up while moving.
        '''
        # If the belt is currently stopped, we can decide to not grab moving
        # if the belt stopped more than X seconds ago
        if not self.moving:
            if time.time() - self.last_moved_time > TASK3B_THRESHOLD:
                self.grab_moving = False

        # If the belt is currently moving, we can decide to grab moving if the
        # last period of stopping was less than X seconds
        else:
            if self.last_stopping_duration < TASK3B_THRESHOLD:
                self.grab_moving = True
    
    def box_distance(self, box: Box) -> float:
        '''
        Returns the distance of the given box from the fixed frame.
        '''
        return np.linalg.norm(np.array([box.x, box.y]))

    def box_rel_zrot(self, box: Box) -> float:
        '''
        Returns the magnitude of the relative rotation of the given box
        to the end effector in rads.
        '''
        ang = np.arctan(box.y / box.x)
        #rospy.loginfo(f'^%& {np.rad2deg(ang) = } deg')
        return abs(ang - box.zrot)

    def heuristic_relative_rotation(self, angle: float) -> float:
        '''
        Returns a float between 0 and 1 corresponding to the heuristic
        score of the given relative rotation angle.
        '''
        #rospy.loginfo(f'heuristic angle: {np.rad2deg(angle)} deg')
        angle = angle % (np.pi/2) # set angle between 0 and 90 deg
        return float('-inf') if np.deg2rad(25) < angle < np.deg2rad(65) else 1

    def grab_best(self) -> str:
        '''
        Applies various heuristics to determine the ID of the box most
        ideal to pick up.
        '''
        id_scores = []
        for box_id in self.boxes:
            box = self.boxes[box_id]
            # Distance heuristic inversely scales box distance by max. dist.
            h_dist = 1 - self.box_distance(box) / 0.31
            # Relative rotation heuristic scales relative rotation such that
            # multiples of 90 deg scale to 1 and multiples of 45 deg to 0
            box_zrot = self.box_rel_zrot(box)
            h_zrot = self.heuristic_relative_rotation(box_zrot)
            id_scores.append((box_id, h_dist + h_zrot))

        best_id, best_score = sorted(id_scores, key=lambda t: t[1], reverse=True)[0]
        rospy.loginfo(f'heuristic score: {best_score} for ID={best_id}')
        return best_id if best_score > 0 else None

    def pickup_block(self, box_id: str) -> int:
        '''
        Commands joints and end effector to pick up the box with the given ID.
        If 
        '''
        # Get the current coordinates of the desired box
        box = self.boxes[box_id]; x = box.x; y = box.y
         
        # Pre-emptively check if it will be possibly to grab
        z = GRABBY_HEIGHT
        coords = (x, y, z)
        possible_grab = self.desired_pos_publisher(coords, rad_offset=RAD_OFFSET, dry=True)
        if not possible_grab:
            return STATE_FIND
        # Move to above box
        z = EMPTY_HEIGHT
        coords = (x, y, z)
        possible = self.desired_pos_publisher(coords, rad_offset=RAD_OFFSET)
        if not possible:
            return STATE_FIND
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        # Move to grab box
        z = GRABBY_HEIGHT
        coords = (x, y, z)
        possible = self.desired_pos_publisher(coords, rad_offset=RAD_OFFSET)
        # We already checked it was possible to no need to check
        if not possible:
            rospy.logwarn('Pre-emptive check failed')
            return STATE_RESET
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        # Allow it to stabilise
        time.sleep(0.2)
        # Grab
        self.command_gripper(open_gripper=False)
        time.sleep(GRAB_TIME)
        # Move up
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
        # Delete old boxes
        self.delete_old_boxes()
        # Act on state and update state
        self.state = self.state_funcs[self.state]()

    def move_to(self, coords, pitch: float, rad_offset: int = 0, error_tol: float = ERROR_TOL) -> bool:
        '''
        A wrapper around desired_pos_publisher that handles waiting for the
        end effector to reach the desired position.
        '''
        self.desired_pos_publisher(coords, pitch, rad_offset)
        while self.position_error > error_tol and not rospy.is_shutdown():
            time.sleep(0.01)

    def state_reset(self):
        '''Moves the robot into the idle position with the gripper open.'''
        self.command_gripper(open_gripper=True)
        thetas = Thetas()
        thetas.thetas = (0, -np.pi/4, np.pi*3/4, np.pi/2)
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
        rospy.loginfo(f'state_find: boxes exist')

        #while self.moving and self.grab_moving == False:
        #    rospy.loginfo(f'state_find: waiting for belt to stop')
        #    time.sleep(0.5)
            
        rospy.loginfo(f'state_find: selecting box')
        self.desired_id = self.grab_best()
        if not self.desired_id:
            rospy.loginfo(f'state_find: no suitable box positions')
            time.sleep(0.5)
            return STATE_FIND
        rospy.loginfo(f'state_find: chose box {self.desired_id}')
       
        # if belt is still for more than 10s, assume 3a
        if (self.last_stopped_time - self.last_moved_time) > 10:
            rospy.loginfo(f'state_find: identified task 3a')
            return STATE_GRAB
            
        # if belt has been still for less than 10s and more than 9s
        # assume 1 or 2 and wait for belt to start moving again
        if (self.last_stopped_time - self.last_moved_time) > 9:
            rospy.loginfo(f'state_find: sleeping for a second')
            time.sleep(1)
            return STATE_FIND

        rospy.loginfo(f'state_find: returning')
        return STATE_GRAB

    def state_grab(self):
        '''
        Pick up the block with the ID chosen during the previous state.
        '''
        rospy.loginfo(f'state_grab: starting')
        alt_state = self.pickup_block(self.desired_id)
        if alt_state is not None:
            rospy.loginfo(f'state_grab: failed')
            return alt_state
        rospy.loginfo(f'state_grab: got block')

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

        # To avoid collision, from the colour detection pose first rotate the
        # base to the x, y coordinate of the desired drop-off zone
        #p = np.array(coords)
        #x, y = np.linalg.norm([0.1, 0.1]) / np.linalg.norm(p) * p
        #coords = (x, y, CARRY_HEIGHT)
        coords = (coords[0], coords[1], CARRY_HEIGHT)
        # coords = (-0.1, 0.1, COLOUR_DETECT_HEIGHT)
        self.move_to(coords, pitch=-np.pi/2, error_tol=ERROR_TOL_COARSE)

        # Move end-effector directly down to place the block
        coords = (coords[0], coords[1], DROPOFF_HEIGHT)
        self.move_to(coords, pitch=-np.pi/2)

        # Release the block
        time.sleep(0.3)
        self.command_gripper(open_gripper=True)
        time.sleep(GRAB_TIME)

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
