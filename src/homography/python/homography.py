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

class Homography:
    def __init__(self):
        self.init_coords()

    def start(self):
        self.image_pub = rospy.Publisher("homography_output", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("homography_input", Image, self.callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def init_coords(self):
        img_coords = np.array([[231, 553], [697, 310], [910, 302], [1225, 369]])
        utm_coords = np.array([
            [586417.3794014237, 6138234.964123087],
            [586539.4879360864, 6138145.410182816],
            [586535.4632636021, 6138085.66494489],
            [586491.0179540929, 6138018.670434019]])
        
        for coord in utm_coords:
            coord[0] -= 586380
            coord[0] *= 8
            coord[1] -= 6137980
            coord[1] *= 8

        ret, mask = cv2.findHomography(img_coords, utm_coords)
        self.transform = ret

    def callback(self, data):
        cv_image = self.to_opencv_image(data)

        cv_image = self.transform_image(cv_image)

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

    def transform_image(self, cv_image):
        cv_image = cv2.warpPerspective(cv_image, self.transform, (2000, 2000))
        cv_image = cv2.flip(cv_image, 0)
        return cv_image

def main():
    rospy.init_node('homography')
    print("Launching homography node")
    node = Homography()
    node.start()

if __name__ == '__main__':
    main()
