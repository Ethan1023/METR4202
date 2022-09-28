#!/usr/bin/env python3

import modern_robotics as mr
import numpy as np
import rospy

from scipy.spatial.transform import Rotation

from msg import BoxTransform
from fiducial_msgs import FiducialTransform
from geometry_msgs import Transform, Vector3, Quaternion

from constants import T_CAMERA_TO_FIXED


class BoxDetector:
    def __init__(self, publisher_queue: int = 10) -> None:
        rospy.init_node('box_detection', anonymous=False)
        self.publisher_queue = publisher_queue
        self.publisher = rospy.Publisher('box_transforms', Transform,
            queue_size=publisher_queue)

    def publish(self, transform: Transform) -> None:
        '''
        Publish to the "box_transforms" topic a Transform object representing
        a box position in the fixed frame.
        '''
        self.publisher.publish(transform)

    def subscribe(self) -> None:
        '''Subscribes to the "fiducial_transforms" topic.'''

        def callback(fiducial_transform: FiducialTransform) -> None:
            '''
            Converts a FiducialTransform object (camera frame) into a
            BoxTransform object (fixed frame) and publishes it.
            '''
            # Unpack FiducialTransform message object
            fiducial_id = fiducial_transform.fiducial_id
            transform = fiducial_transform.transform

            # Transform from camera frame to fixed frame
            B = self.Transform_to_SE3(transform)
            T = np.array(T_CAMERA_TO_FIXED)
            S = self.SE3_to_Transform(T @ B)

            # Publish BoxTransform message object
            self.publish(BoxTransform(fiducial_id=fiducial_id, transform=S))

        rospy.Subscriber('fiducial_transforms', FiducialTransform, callback)
        rospy.spin()

    def Transform_to_SE3(transform: Transform) -> np.array:
        '''Convert a Transform message object into an SE3 matrix.'''
        p = transform.translation; p = np.array([p.x, p.y, p.z])
        R = transform.rotation; R = Rotation.from_quat([R.x, R.y, R.z, R.w])
        return mr.RpToTrans(R.as_matrix(), p)

    def SE3_to_Transform(SE3: np.array) -> Transform:
        '''Convert an SE3 matrix into a Transform message object.'''
        R, p = mr.TransToRp(SE3);
        p = Vector3(*p)
        R = Rotation.from_matrix(R).as_quat(); R = Quaternion(*R)
        return Transform(translation=p, rotation=R)

    def run(self):
        '''Runs the box_detection node.'''
        self.subscribe()


if __name__ == '__main__':
    BoxDetector.run()
