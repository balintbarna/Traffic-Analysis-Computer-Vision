#!/usr/bin/env python2.7
from __future__ import print_function

#import roslib
#roslib.load_manifest('homography')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Homography:
    def __init__(
            self,
            img_coords=None,
            utm_coords=None,
            coord_scale_offset=None,
            input_topic="/input",
            output_topic="/output"
    ):
        self.img_coords = img_coords
        self.utm_coords = utm_coords
        self.coord_scale_offset = coord_scale_offset

        if not (
                self.img_coords is None or
                self.utm_coords is None or
                self.coord_scale_offset is None
        ):
            self.init_coords()

        self.input_topic = input_topic
        self.output_topic = output_topic

    def start(self):
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def init_coords(self):
        self.img_coords = np.array(self.img_coords)
        self.utm_coords = np.array(self.utm_coords)
        #self.img_coords = np.array([[231, 553], [697, 310], [910, 302], [1225, 369]])
        #self.utm_coords = np.array([
        #    [586417.3794014237, 6138234.964123087],
        #    [586539.4879360864, 6138145.410182816],
        #    [586535.4632636021, 6138085.66494489],
        #    [586491.0179540929, 6138018.670434019]])
        
        for coord in self.utm_coords:
            coord[0] -= self.coord_scale_offset[0]
            coord[0] *= self.coord_scale_offset[1]
            coord[1] -= self.coord_scale_offset[2]
            coord[1] *= self.coord_scale_offset[3]

        ret, mask = cv2.findHomography(self.img_coords, self.utm_coords)
        self.transform = ret

    def callback(self, data):
        cv_image = self.to_opencv_image(data)

        if not (
                self.img_coords is None or
                self.utm_coords is None or
                self.coord_scale_offset is None
        ):
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
        cv_image = cv2.warpPerspective(cv_image, self.transform, (cv_image.shape[1], cv_image.shape[0]))
        return cv_image

def main():
    rospy.init_node('homography')
    print("Launching homography node")
    node = Homography()
    node.start()

if __name__ == '__main__':
    main()
