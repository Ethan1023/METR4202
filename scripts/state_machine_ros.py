import rospy
import time
import numpy as np

from std_msgs.msg import Bool, ColorRGBA, Float32
#from sensor_msgs.msg import JointState
from metr4202.msg import BoxTransformArray, Pos, GripperState  # Custom messages from msg/
from inverse_kinematics import inv_kin

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
        self.ids = []
        self.xs = []
        self.yx = []
        self.zrots = []
        # TODO - variables to store current state
            # i.e. placing block, picking up block, returning etc

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
        # TODO - populate class variables
        # TODO - track position history and time to get velocity?
        for id in None:
            if id in self.ids:
                i = self.ids.index(id)
                self.xs[i] = None
                self.ys[i] = None
                self.zrots[i] = None
            else:
                self.ids.append(id)
                self.xs.append(None)
                self.ys.append(None)
                self.zrots.append(None)
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

    def loop(self):
        '''
        State machine main loop
        read current state
        use block position and/or position error to change state or issue command to gripper or joint_controller
        '''
        # do logic
        if len(self.ids) == 1:
            state_1(self.xs[0], self.ys[0])
        pass
        # publsh commands if needed

