#!/usr/bin/env python3

import rospy
import numpy as np
from inverse_kinematics import inv_kin
from metr4202.msg import Pos, Thetas

class inverse_kinematics:
    def __init__(self):
        self.pub = rospy.Publisher('desired_theta', Thetas, queue_size=1)
        rospy.init_node('inv_kin', anonymous=False)
        rospy.Subscriber('desired_pos', Pos, self.callback)
        

    def callback(self, pos):
        coords = np.array([pos.x, pos.y, pos.z])
        thetas = inv_kin(coords, pos.pitch)
        msg = Thetas()
        msg.thetas = thetas
        self.pub.publish(msg)

def main():
    ik = inverse_kinematics()
    rospy.spin()

if __name__ == '__main__':
    main()
