#!/usr/bin/env python3

'''
This script subscribes to the "test_colour" topic for colour detection and
and the "request_colour" topic for incoming detection requests.

When an incoming request is received, the most recently detected colour is
published to the "box_colour" topic.

It is run by calling:
$ rosrun metr4202 colour_detection.py

When the script is running, it can be tested in another terminal by the
built-in test function by running:
$ rosrun metr4202 colour_detection.py --test

This publishes requests every second and prints the value published in
response (to the terminal).
'''

import numpy as np
import rospy

from std_msgs.msg import Bool, ColorRGBA, String


class ColourDetection:
    def __init__(self, publisher_queue: int = 10) -> None:
        rospy.init_node('colour_detection', anonymous=False)
        self.publisher_queue = publisher_queue
        self.publisher = rospy.Publisher('box_colour', String,
            queue_size=publisher_queue)

    def classify_colour(self, colour: ColorRGBA) -> str:
        '''
        Classifies given colour as: "red", "green", "blue", "yellow" or "other".
        '''
        r = colour.r; g = colour.g; b = colour.b; rgb = np.array([r, g, b])

        # Avoid misclassifying colour due to occasional cv error
        if any(rgb == 0):
            return 'other'

        # Get the relative magnitude of each colour component
        cmax = max(rgb); r, g, b = rgb / cmax
       

        # Classify the colour
        rospy.loginfo(f'{r = }, {g = }, {b = }')
        if (r > 0.95 and g < 0.4 and b < 0.3):
            return 'red'

        if (r < 0.55 and g > 0.95 and b < 0.5):
            return 'green'

        if (r < 0.5 and g > 0.8 and b > 0.95):
            return 'blue'

        if (r > 0.95 and g > 0.8 and b < 0.4):
            return 'yellow'

        return 'other'

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
            # Request probably won't be "false"...
            self.publisher.publish(self.classify_colour(self.detected_colour))

        def update_callback(colour: ColorRGBA) -> None:
            '''
            Updates the "self.detected_colour" instance variable with the most
            recently detected colour.
            '''
            self.detected_colour = colour

        rospy.Subscriber('request_colour', Bool, response_callback)
        rospy.Subscriber('test_color', ColorRGBA, update_callback)
        rospy.spin()

    def run(self) -> None:
        '''Runs the colour_detection node.'''
        print('colour_detection ready')
        self.subscribe()

    @staticmethod
    def test() -> None:
        '''
        Creates a new node that periodically publishes requests to
        "request_colour" and subscribes to (and prints) the response.
        '''
        import time

        def callback(colour: String) -> None:
            print(f'Detected colour:', colour)

        rospy.init_node('test_colour_detection', anonymous=False)
        rospy.Subscriber('box_colour', String, callback)

        publisher = rospy.Publisher('request_colour', Bool, queue_size=1)
        request = Bool(); request.data = True

        requests_sent = 0
        while not rospy.is_shutdown():
            publisher.publish(request)
            print(f'[x] Sent request {requests_sent+1}')
            requests_sent += 1
            time.sleep(1)


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == '--test':
            ColourDetection.test()
            sys.exit(0)

    ColourDetection().run()
