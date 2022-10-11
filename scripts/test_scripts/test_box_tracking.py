#!/usr/bin/env python3

'''
This script currently tests the following nodes:

    - "joint_controllers"
    - "gripper_controller"

It does so by subscribing to the "box_transform" topic and publishing the
position of the first block to "desired_pos". A constant offset is added to the
position to keep the camera unobstructed and the pitch is kept at a constant
90 deg. The end effector should approximately track the position of a block.

It also publishes open and grip states to the gripper controller. The gripper
should open before moving and close after moving.

It is run by calling:
$ rosrun metr4202 test_box_tracking.py
'''

import numpy as np
import rospy

from metr4202.msg import BoxTransformArray, GripperState, Pos


class BoxTracker():
    def __init__(self, publisher_queue_size: int = 10) -> None:
        rospy.init_node('box_tracking', anonymous=False)
        self.publisher_queue_size = publisher_queue_size

        self.joint_state_publisher = rospy.Publisher('desired_pos', Pos,
            queue_size=publisher_queue_size)

        self.gripper_state_publisher = rospy.Publisher('gripper_state',
            GripperState, queue_size=self.publisher_queue_size)

    def publish_joint_state(self, pos: Pos) -> None:
        '''
        Publish to the "desired_pos" topic a Pos message object representing
        the desired end effector configuration in the fixed frame.
        '''
        self.joint_state_publisher.publish(pos)

    def publish_gripper_state(self, gripper_state: GripperState) -> None:
        '''
        Publish to the "gripper_state" topic a GripperState message object
        representing the desired gripper state: open or grip.
        '''
        self.gripper_state_publisher.publish(gripper_state)

    def subscribe(self) -> None:
        '''Subscribes to the "box_transforms" topic.'''

        def callback(box_transforms: BoxTransformArray) -> None:
            '''
            Converts the first BoxTransform object in the given array message
            into a position and pitch and publishes it as a Pos message object.
            '''
            # When a transform is received, open the gripper
            gripper_state = GripperState(); gripper_state.open = True
            self.publish_gripper_state(gripper_state)

            # Extract the position of a block from the transform array
            transforms = box_transforms.transforms

            if not transforms: # if there are no messages then exit
                return

            box_transform = transforms[0]

            T = box_transform.transform.translation # Vector3

            # Add constant offset to position so the camera is unobstructed
            x = T.x - 0.05; y = T.y; z = 0.1 # [m]

            # Create and publish Pos message object
            pos = Pos(); pos.x = x; pos.y = y; pos.z = z; pos.pitch = -np.pi/2
            self.publish_joint_state(pos)

            # When the sequence is complete, close the gripper
            gripper_state = GripperState(); gripper_state.open = False;
            self.publish_gripper_state(gripper_state)

        rospy.Subscriber('box_transforms', BoxTransformArray, callback)
        rospy.spin()

    def run(self) -> None:
        '''Runs the block_tracking node.'''
        print('block_tracking ready')
        self.subscribe()


if __name__ == '__main__':
    BoxTracker().run()
