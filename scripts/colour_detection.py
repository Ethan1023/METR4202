#!/usr/bin/env python3

'''
TODO: documentation
'''

import rospy

from std_msgs.msg import Bool, ColorRGBA


class ColourDetection:
    def __init__(self, publisher_queue: int = 10) -> None:
        self.publisher_queue = publisher_queue
        self.publisher = rospy.Publisher('box_colour', ColorRGBA,
            queue_size=publisher_queue)

    def subscribe(self) -> None:
        '''
        Subscribes to both the "test_colour" and the "request_colour" topics.
        '''
        # Initialise instance variable to store most recently detected colour
        self.detected_colour = None

        def response_callback(request: Bool) -> None:
            '''
            Publishes the colour in the "self.detected_colour" variable to
            the "box_colour" topic.
            '''
            return self.colour # request probably won't be False...

        def update_callback(colour: ColorRGBA) -> None:
            '''
            Updates the "self.detected_colour" instance variable with the most
            recently detected colour.
            '''
            self.detected_colour = colour

        rospy.Subscriber('colour_request', Bool, response_callback)
        rospy.Subscriber('test_colour', ColorRGBA, update_callback)
        rospy.spin()

    def run(self):
        '''Runs the colour_detection node.'''
        print('colour_detection ready')
        self.subscribe()


if __name__ == '__main__':
    ColourDetection().run()
