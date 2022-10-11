import rospy
import time
import numpy as np
#from sensor_msgs.msg import JointState
from metr4202.msg import Pos  # Custom messages from msg/

class StateMachine:
    def __init__(self):
        self.other_init()
        self.rospy_init()
        print(f'StateMachine initialised')

    def other_init(self):
        #self.theta_stale = True
        self.printing = True
        self.camera_stale = True
        self.position_error = 0
        # Assume joint controller runs faster than camera so no real need to track is position error is stale
        # TODO - camera input variables
        # TODO - variables to store current state
            # i.e. placing block, picking up block, returning etc

    def rospy_init(self):
        # Create node
        rospy.init_node('state_machine', anonymous=False)
        # Publish to desired end effector
        self.joint_pub = rospy.Publisher('desired_pos', Pos, queue_size=1)
        # Subscribe to camera - TODO
        rospy.Subscriber(None, None, self.camera_callback)
        # Subscribe to angle error - TODO
        rospy.Subscriber('position_error', None, self.position_error_callback)

        while self.camera_stale and not rospy.is_shutdown():
            # Block operation until camera data received
            time.sleep(0.01)

    def camera_callback(self, msg):
        # TODO - populate class variables
        self.camera_stale = False

    def position_error_callback(self, msg):
        # TODO -  populate class variable
        pass

    def run(self):
        while not rospy.is_shutdown():
            while self.camera_stale and not rospy.is_shutdown():
                time.sleep(0.001)
            if rospy.is_shutdown():
                return 0
            self.camera_stale = True
            self.loop()

    def loop(self):
        '''
        State machine main loop
        read current state
        use block position and/or position error to change state or issue command to gripper or joint_controller
        '''

