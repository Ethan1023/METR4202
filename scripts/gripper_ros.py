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
        False: 1375, # grip
        True:  2000, # open
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
            print('gripper do:', end='')
            print('open' if state else 'grip')
            self.rpi.set_servo_pulsewidth(self.rpi_pin, self.position[state])

        rospy.Subscriber('gripper_state', GripperState, callback)
        rospy.spin()

    def run(self) -> None:
        '''Runs the gripper_controller node.'''
        print('gripper_controller ready')
        self.subscribe()

    @staticmethod
    def test() -> None:
        '''
        Creates a new node that publishes to "gripper_state" the
        commands entered on the command line.
        '''
        print('Test gripper_controller node by typing "open" or "grip". Type "exit" to finish.')

        rospy.init_node('test_gripper_controller', anonymous=False)
        publisher = rospy.Publisher('gripper_state', GripperState, queue_size=0)

        while True:
            cmdstr = input('Type "open" or "grip" (or "exit"): ')

            if cmdstr in ('exit'):
                sys.exit(0)

            if cmdstr in ('open', 'grip'):
                open = {'open': 1, 'grip': 0}[cmdstr]
                gripper_state = GripperState(); gripper_state.open = open
                publisher.publish(gripper_state)


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == '--test':
            try:
                GripperController.test()
            except KeyboardInterrupt:
                pass
            sys.exit(0)

    GripperController().run()
