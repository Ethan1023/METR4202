#!/usr/bin/env python3
import rospy
import pigpio
from metr4202.msg import Gripper_OpenClose  # Custom messages from msg/

class Gripper:
    def __init__(self):
        # Initialise ROS node - anonymouse means the same node can't be run twice atst (I think)
        rospy.init_node('gripper', anonymous=False)
        # Subscribe to 'gripper_openclose' to receive desired config as Gripper_OpenClose message, and call self.callback with this message
        rospy.Subscriber('gripper_openclose', Gripper_OpenClose, self.callback)
        self.rpi = pigpio.pi()
        self.rpi.set_mode(18, pigpio.OUTPUT)
        self.CLOSE = 1000
        self.GRIP = 1500
        self.OPEN = 2000

    def callback(self, msg):
        # Extract position from message
        open_gripper = msg.open_gripper
        if open_gripper:
            # Open gripper
            self.rpi.set_servo_pulsewidth(18, self.OPEN)
        else:
            # Close gripper
            self.rpi.set_servo_pulsewidth(18, self.GRIP)

def main():
    # Create ROS node
    gripper = Gripper()
    # Prevent python from exiting
    rospy.spin()

if __name__ == '__main__':
    main()
