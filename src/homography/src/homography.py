#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('homography')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class homography_node:
    def __init__(self):
        rospy.init_node('homography')
        self.image_pub = rospy.Publisher("homography_output", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("homography_input", Image, self.callback)

    def callback(self, data):
        cv_image = self.to_opencv_image(data)

        cv_image = self.transform_image(cv_image, None, None)

        self.publish_image(cv_image)

    def to_opencv_image(self, data):
        try:
            return self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as ex:
            print(ex)

    def to_ros_image(self, cv_image):
        try:
            return self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as ex:
            print(ex)

    def publish_image(self, cv_image):
        ros_image = self.to_ros_image(cv_image)
        try:
            self.image_pub.publish(ros_image)
        except Exception as ex:
            print(ex)

    def transform_image(self, cv_image, img_points, real_points):
        return cv_image

def main():
    print("Launching homography node")
    node = homography_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
