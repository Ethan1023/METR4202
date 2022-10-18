#!/usr/bin/env python3

import rospy
import numpy as np
from inverse_kinematics import inv_kin
from metr4202.msg import Pos, Thetas  # Custom messages from msg/

'''
Inverse kinematics node
I don't think it has to be a class, just bare methods and global variables probably work (emphasis on probably)

This node will listen to the 'desired_pos' topic for a Pos message, and upon receiving a message it will immediately
calculate the angles required and publish this to the 'desired_theta' topic as a Thetas message.
'''

class inverse_kinematics:
    def __init__(self):
        # Publish desired angles to 'desired_theta' using Thetas message type
        # Queue size of 1 means that if subscriber falls behind the old value will be overwritten with new value
        self.pub = rospy.Publisher('desired_theta', Thetas, queue_size=1)
        # Initialise ROS node - anonymouse means the same node can't be run twice atst (I think)
        rospy.init_node('inv_kin', anonymous=False)
        # Subscribe to 'desired_pos' to receive desired positions as Pos message, and call self.callback with this message
        rospy.Subscriber('desired_pos', Pos, self.callback)
        

    def callback(self, pos):
        # Extract coordinates from message
        coords = np.array([pos.x, pos.y, pos.z])
        # Calculate desired thetas
        thetas = inv_kin(coords, pos.pitch)
        # Initialise message
        msg = Thetas()
        # Populate message
        msg.thetas = thetas
        # Publish message
        self.pub.publish(msg)

def main():
    # Create ROS node
    ik = inverse_kinematics()
    # Prevent python from exiting
    rospy.spin()

if __name__ == '__main__':
    main()
