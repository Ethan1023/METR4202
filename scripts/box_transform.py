#!/usr/bin/env python3

'''
This script subscribes to the "fiducial_transforms" topic of the ArUco tag
detection library and publishes to the "box_transforms" topic.

It is run by calling:
$ rosrun metr4202 box_transform.py

When the script is running, the output can be visualised in another terminal:
$ rosrun metr4202 box_transform.py --test

The "fiducial_transforms" topic publishes "FiducialTransformArray" objects, which
are an array of transformations representing visible ArUco tag positions in the
camera frame.

The script converts these transformations into the fixed frame using the
transformation matrix "T_CAMERA_TO_FIXED" defined in "constants.py".

The transformed positions are published as a "BoxTransformArray" message to the
"box_transforms" topic in a largely analogous format to "fiducial_transforms".
'''

import modern_robotics as mr
import numpy as np
import rospy

from scipy.spatial.transform import Rotation

from metr4202.msg import BoxTransform, BoxTransformArray
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Transform, Vector3, Quaternion

from constants import T_CAMERA_TO_FIXED


class BoxTransform:
'''
This class holds all the functions that subscribe to the fiducial transforms topic of
the ArUco tag detection library and publishes to the box transforms topic. It
also converts the ArUco tag values to messages.
'''
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

                # Create BoxTransform message object
                box_transform = BoxTransform()
                box_transform.fiducial_id = fiducial_id
                box_transform.transform = S

                box_transforms.append(box_transform)

            # Publish BoxTransformArray message object
            box_transform_array = BoxTransformArray()
            box_transform_array.transforms = box_transforms
            self.publish(box_transform_array)

        rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback)
        rospy.spin()

    def Transform_to_SE3(self, transform: Transform) -> np.array:
        '''Convert a Transform message object into an SE3 matrix.'''
        p = transform.translation; p = np.array([p.x, p.y, p.z])
        R = transform.rotation; R = Rotation.from_quat([R.x, R.y, R.z, R.w])
        return mr.RpToTrans(R.as_matrix(), p)

    def SE3_to_Transform(self, SE3: np.array) -> Transform:
        '''Convert an SE3 matrix into a Transform message object.'''
        R, p = mr.TransToRp(SE3);
        p = Vector3(*p)
        R = Rotation.from_matrix(R).as_quat(); R = Quaternion(*R)
        transform = Transform(); transform.translation = p; transform.rotation = R
        return transform

    def run(self) -> None:
        '''Runs the box_detection node.'''
        print('box_transform ready')
        self.subscribe()

    @staticmethod
    def test() -> None:
        '''
        Creates a new node that subscribes to "box_transforms" and prints
        the output.
        '''
        def callback(box_transforms: BoxTransformArray) -> None:
            print(box_transforms)

        rospy.init_node('test_box_transform', anonymous=False)
        rospy.Subscriber('box_transforms', BoxTransformArray, callback)
        rospy.spin()


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == '--test':
            BoxTransform.test()
            sys.exit(0)

    BoxTransform().run()
