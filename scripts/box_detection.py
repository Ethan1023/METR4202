#!/usr/bin/env python

import modern_robotics as mr
import numpy as np
import rospy

from scipy.spatial.transform import Rotation

from fiducial_msgs import FiducialTransforms
from geometry_msgs import Transform

from constants import T_CAMERA_TO_FIXED


class BoxDetector:
    def __init__(self, publisher_queue: int = 10) -> None:
        rospy.init_node('box_detection', anonymous=False)
        self.publisher_queue = publisher_queue
        self.publisher = rospy.Publisher('box_transforms', Transform,
            queue_size=publisher_queue)

    def publish(self, transform: Transform) -> None:
        ''''''
        self.publisher.publish(transform)

    def subscribe(self) -> None:
        ''''''

        def callback(data) -> None:
            ''''''
            p = data.translation; p = np.array([p.x, p.y, p.z])
            R = data.rotation; R = Rotation.from_quat([R.x, R.y, R.z, R.w])
            B = mr.RpToTrans(R.as_matrix(), p)
            T = np.array(T_CAMERA_TO_FIXED)
            self.publish(T @ B)

        rospy.Subscriber('fiducial_transforms', FiducialTransforms, callback)
        rospy.spin()

    def run(self):
        ''''''
        self.subscribe()


if __name__ == '__main__':
    BoxDetector.run()
