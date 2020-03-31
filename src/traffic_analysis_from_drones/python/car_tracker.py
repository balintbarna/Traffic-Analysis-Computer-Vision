#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('traffic_analysis_from_drones')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CarTracker():
    def __init__(self):
        pass

    def analyze_frame(self, frame):
        output = cv2.circle(frame, (100, 100), 30, (0, 0, 255), 2)
        return output


class car_tracker_node:
    def __init__(self):
        rospy.init_node('car_tracker')
        self.image_pub = rospy.Publisher("tracked_cars", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("stabilized_frame", Image, self.callback)

        self.car_tracker = CarTracker()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = self.analyze_image(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(e)
        
    def analyze_image(self, image):
        image = self.car_tracker.analyze_frame(image)
        return image


def main(args):
    ic = car_tracker_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    print("Launching the car tracker")
    main(sys.argv)

