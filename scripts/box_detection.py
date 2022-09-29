#!/usr/bin/env python3

import modern_robotics as mr
import numpy as np
import rospy

from scipy.spatial.transform import Rotation

from METR4202.msg import BoxTransform, BoxTransformArray
from fiducial_msgs import FiducialTransform, FiducialTransformArray
from geometry_msgs import Transform, Vector3, Quaternion

from constants import T_CAMERA_TO_FIXED


class BoxDetector:
    def __init__(self, publisher_queue: int = 10) -> None:
        rospy.init_node('box_detection', anonymous=False)
        self.publisher_queue = publisher_queue
        self.publisher = rospy.Publisher('box_transforms', BoxTransformArray,
            queue_size=publisher_queue)

    def publish(self, box_transforms: BoxTransformArray) -> None:
        '''
        Publish to the "box_transforms" topic a BoxTransformArray message
        object representing box positions in the fixed frame.
        '''
        self.publisher.publish(box_transforms)

    def subscribe(self) -> None:
        '''Subscribes to the "fiducial_transforms" topic.'''

        def callback(fiducial_transforms: FiducialTransformArray) -> None:
            '''
            Converts an array of FiducialTransform objects (camera frame)
            into an array of BoxTransform objects (fixed frame) and
            publishes it.
            '''
            box_transforms = []
    
            for fiducial_transform in fiducial_transforms.transforms:
                # Unpack FiducialTransform message object
                fiducial_id = fiducial_transform.fiducial_id
                transform = fiducial_transform.transform

                # Transform from camera frame to fixed frame
                B = self.Transform_to_SE3(transform)
                T = np.array(T_CAMERA_TO_FIXED)
                S = self.SE3_to_Transform(T @ B)

                box_transforms.append(BoxTransform(fiducial_id=fiducial_id,
                    transform=S))                
    
            # Publish BoxTransformArray message object
            self.publish(BoxTransformArray(transforms=box_transforms))

        rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback)
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
