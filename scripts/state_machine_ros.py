#!/usr/bin/env python3

'''
TODO: documentation
'''

import rospy
import time
import numpy as np

from std_msgs.msg import Bool, ColorRGBA, Float32
#ColorRGBA = None
from metr4202.msg import BoxTransformArray, Pos, GripperState  # Custom messages from msg/
from inverse_kinematics import inv_kin
from constants import EMPTY_HEIGHT, GRABBY_HEIGHT, CARRY_HEIGHT, ERROR_TOL, GRAB_TIME, \
                      STATE_RESET, STATE_FIND, STATE_GRAB, STATE_COLOUR, STATE_PLACE, STATE_ERROR, \
                      L1, L2, L3, L4, PLACE_DICT, VELOCITY_AVG_TIME, OMEGA_THRESHOLD, BASE_TO_BELT, STATE_NAMES
from maths import yaw_from_quat

class StateMachine:
    def __init__(self, dorospy = True):
        self.other_init()
        if dorospy:
            self.rospy_init()
        print(f'StateMachine initialised')

    def other_init(self):
        #self.theta_stale = True
        self.printing = True
        self.camera_stale = True
        self.position_error = 0
        # Assume joint controller runs faster than camera so no real need to track is position error is stale
        # TODO - camera input variables
        self.ids = []
        self.xs = []
        self.ys = []
        self.zrots = []

        self.x_hist = []  # List of lists of position history
        self.y_hist = []
        self.t_hist = []  # List of lists of update times
        self.x_vels = []  # List of velocities
        self.y_vels = []
        self.omegas = []  # list of angular velocities
        self.radii = []  # list of radii
        self.omega = 0    # average angular velocity (0 if not known)
        self.start_time = time.time()   # update while any blocks are not moving
        self.stop_time = time.time()    # update while any blocks are moving

        # variables to store current state
        self.state = STATE_RESET
        self.state_funcs = (self.state_reset, self.state_find, self.state_grab, \
                            self.state_colour, self.state_place, self.state_error)
        self.desired_id = None

    def rospy_init(self):
        # Create node
        rospy.init_node('state_machine', anonymous=False)
        # Publish to desired end effector
        self.pos_pub = rospy.Publisher('desired_pos', Pos, queue_size=1)
        # Publish to request box colour
        self.colour_pub = rospy.Publisher('request_colour', Bool, queue_size=1)
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

    def camera_callback(self, msg):
        for box_transform in msg.transforms:
            box_id = box_transform.fiducial_id
            if box_id in self.ids:
                i = self.ids.index(box_id)
                self.xs[i] = box_transform.transform.translation.x
                self.ys[i] = box_transform.transform.translation.y
                self.zrots[i] = yaw_from_quat(box_transform.transform.rotation)
                self.radii[i] = np.sqrt((self.xs[i]-BASE_TO_BELT)**2 + self.ys[i]**2)
                # Update position 'queue'
                self.x_hist[i].append(self.xs[i])
                self.y_hist[i].append(self.ys[i])
                self.t_hist[i].append(time.time())
            else:
                print(f'New box found!')
                self.ids.append(box_id)
                self.xs.append(box_transform.transform.translation.x)
                self.ys.append(box_transform.transform.translation.y)
                self.zrots.append(yaw_from_quat(box_transform.transform.rotation))
                self.radii.append(np.sqrt((self.xs[-1]-BASE_TO_BELT)**2 + self.ys[-1]**2))
                # Create 'queue' with new position
                self.x_hist.append([self.xs[-1]])
                self.y_hist.append([self.ys[-1]])
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
                    while t_hist[-1] - t_hist[1] > VELOCITY_AVG_TIME:
                        del x_hist[0]
                        del y_hist[0]
                        del t_hist[0]
                    # Calculate velocities
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
            self.stop_time = time.time()
            print(f'Time since started = {self.stop_time - self.start_time} s')
        else:
            self.start_time = time.time()
            print(f'Time since stopped = {self.start_time - self.stop_time} s')
        if len(self.ids) > 0:
            self.camera_stale = False

    def colour_detect_callback(self, data: ColorRGBA) -> None:
        '''
        Updates the self.detected_colour variable with the received response.
        '''
        self.detected_colour = data

    def request_colour(self) -> ColorRGBA:
        '''
        Publishes a request to the "colour_request" topic and subscribes to the
        "box_colour" topic to receive the response. Returns ColorRGBA.
        '''
        self.detected_colour = None
        request = Bool(); request.data = True
        self.colour_pub.publish(request)

        while not self.detected_colour:
            time.sleep(0.01)
        return self.detected_colour

    def position_error_callback(self, msg):
        # TODO - track change in position error to see if robot is still moving
        self.position_error = msg.data

    def desired_pos_publisher(self, coords, pitch=None):
        '''
        Accept coords and optional pitch
        Return if command was issued
        if pitch is not supplied, try as straight down as possible
        '''
        pos = Pos()
        pos.x, pos.y, pos.z = coords
        if pitch is not None:  # if pitch supplied, try to reach it or fail
            if inv_kin(coords, pitch, self_check=False, check_possible=True):
                pos.pitch = pitch
                self.pos_pub.publish(pos)
                return True
            return False
        else:  # Otherwise use default and increment
            for pitch in np.linspace(-np.pi/2, 0, 10):
                if inv_kin(coords, pitch, self_check=False, check_possible=True):
                    pos.pitch = pitch
                    self.pos_pub.publish(pos)
                    return True
            return False

    def gripper_publisher(self, open_grip=True):
        '''
        Open gripper?
        '''
        msg = GripperState()
        msg.open = open_grip
        self.gripper_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            while self.camera_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            self.camera_stale = True
            self.loop()

    def pickup_block(self, block_id):
        i = self.ids.index(block_id)
        x = self.xs[i]
        y = self.ys[i]
        z = EMPTY_HEIGHT
        coords = (x, y, z)
        self.desired_pos_publisher(coords)
        print(self.position_error)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
            print(self.position_error)
        z = GRABBY_HEIGHT
        coords = (x, y, z)
        self.desired_pos_publisher(coords)
        print(2)
        print(self.position_error)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
            print(2)
            print(self.position_error)
        self.gripper_publisher(False)
        time.sleep(GRAB_TIME)
        z = CARRY_HEIGHT
        coords = (x, y, z)
        self.desired_pos_publisher(coords)

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

    def state_reset(self):
        # Returns robot to initial position and opens gripper
        self.gripper_publisher()
        coords = (L4, 0, L1+L2+L3)
        self.desired_pos_publisher(coords, 0)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        return STATE_FIND

    def state_find(self):
        # Locating and choosing which block to pick up
        # TODO: ADD LOGIC
        self.desired_id = self.ids[0] # TEMPORARY, FIX THIS - TODO
        return STATE_GRAB

    def state_grab(self):
        # Picking up a block
        # If fails, open gripper and return to state_find
        self.pickup_block(self.desired_id)
        return STATE_COLOUR

    def state_colour(self):
        # Checking the block colour
        # If fails, open gripper and return to state_find
        print('colour')
        coords = (BASE_TO_BELT, 0, 0.2) # TODO - get position
        pitch = 0
        self.desired_pos_publisher(coords, pitch)
        while self.position_error > ERROR_TOL and not rospy.is_shutdown():
            time.sleep(0.01)
        self.request_colour()
        return STATE_COLOUR # TEMPORARY, FIX THIS - TODO

    def state_place(self):
        # Get destination from state_colour
        # Checking id list until the block arrives at that destination
        # If block appears back in the id/pos list, go to state_find
        # Once position error is low enough, put block down then return to state_reset
        # TODO - create dictionary of tuples within constants file for colours which returns xy
        # TODO - get z values
        x, y = PLACE_DICT[self.detected_colour]
        z = None
        coords = (x, y, z)
        self.desired_pos_publisher(coords)
        while self.position_error > ERROR_TOL:
            time.sleep(0.01)
        z = None
        coords = (x, y, z)
        self.desired_pos_publisher(coords)
        while self.position_error > ERROR_TOL:
            time.sleep(0.01)
        self.gripper_publisher(True)
        time.sleep(GRAB_TIME)
        z = None
        coords = (x, y, z)
        self.desired_pos_publisher(coords)
        return STATE_RESET

    def state_error(self):
        # If called, open gripper and go to state_reset
        self.gripper_publisher()
        return STATE_RESET

if __name__ == '__main__':
    # Create ROS node
    sm = StateMachine()
    # Prevent python from exiting
    sm.run()
    rospy.spin()