#!/usr/bin/env python

import time
import argparse
import roslib
roslib.load_manifest('video_stream')
import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class video_frame_publisher:
    def __init__(self, video_source, framerate):
        rospy.init_node('video_frame_publisher', anonymous=True)
        self.cap = cv2.VideoCapture(video_source)
        self.image_pub = rospy.Publisher("video_stream_output", Image, queue_size=1)
        self.bridge = CvBridge()
        self.timer = rospy.Timer(rospy.Duration(1. / framerate), self.publish_frame)

    def publish_frame(self, event):
        try:
            ret, image = self.cap.read()
            converted_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.image_pub.publish(converted_image)
        except CvBridgeError as e:
            print(e)


def main():
    parser = argparse.ArgumentParser(
        description='Video file to use')
    parser.add_argument(
        '--video_source', type=str, help='path to video file')
    parser.add_argument(
        '--framerate',
        type=int,
        default=20,
        help='framerate during playback')

    options, args = parser.parse_known_args()

    ip = video_frame_publisher(options.video_source, options.framerate)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
