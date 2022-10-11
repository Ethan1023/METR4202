#!/usr/bin/env python3

'''
TODO: documentation
'''

import pigpio
import rospy

from metr4202.msg import GripperState


class GripperController:

    # Define servo positions corresponding to gripper states
    position = {
        0: 1500, # open
        1: 2000, # grip
    }

    def __init__(self) -> None:
        rospy.init_node('gripper_controller', anonymous=False)
        self.rpi = pigpio.pi()
        self.rpi_pin = 18
        self.rpi.set_mode(self.rpi_pin, pigpio.OUTPUT)

    def subscribe(self) -> None:
        '''Subscribes to the "gripper_state" topic.'''

        def callback(gripper_state: GripperState) -> None:
            '''Sets the gripper position based on the GripperState message.'''
            state = gripper_state.open
            self.rpi.set_servo_pulsewidth(self.rpi_pin, self.position[state])

        rospy.Subscriber('gripper_state', GripperState, callback)
        rospy.spin()

    def run(self) -> None:
        '''Runs the gripper_controller node.'''
        print('gripper_controller ready')
        self.subscribe()


if __name__ == '__main__':
    GripperController().run()

