#!/usr/bin/env python2.7

"""
    Module with class ROSImageStream for either publishing or subscribing 
    to OpenCV image streams in ROS.

    Change log: 
    Created     frnyb       20200329
"""

########################################################################
# Imports:

import argparse
import sys

import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

########################################################################
# Classes:

class ROSImageStream:
    def __init__(
            self,
            pub_topic_name="/img_stream",
            sub_topic_name="/img_stream",
            in_bag_name=None,
            queue_size=1
    ):
        self.pub = None
        self.sub = None

        self.pub_topic_name = pub_topic_name
        self.sub_topic_name = sub_topic_name

        self.in_bag_name = in_bag_name

        self.queue_size = queue_size

    def publish_from_video(
            self,
            video_filename,
            loop=False
    ):
        cap = cv2.VideoCapture(video_filename)
        fps = cap.get(cv2.CAP_PROP_FPS)
        n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if self.pub == None:
            self.pub = rospy.Publisher(
                    self.pub_topic_name, 
                    Image,
                    queue_size=self.queue_size
            )

        bridge = CvBridge()

        next_time = rospy.get_rostime().to_sec()
    
        while(True):
            for i in range(n_frames):
                if rospy.is_shutdown():
                    break

                ret, frame = cap.read()

                if ret == True:
                    img_msg = bridge.cv2_to_imgmsg(
                            frame,
                            encoding="bgr8"
                    )
                    self.pub.publish(img_msg)

                    rospy.sleep(next_time - rospy.get_rostime().to_sec())

                    next_time = next_time + 1 / fps

            cap.release()
            
            if not loop:
                break
            
            cap = cv2.VideoCapture(video_filename)

    def publish_single(
            self,
            img,
            frequency=0,
            encoding="bgr8"
    ):
        if self.pub == None:
            self.pub = rospy.Publisher(
                    self.pub_topic_name, 
                    Image,
                    queue_size=self.queue_size
            )

        rate = None
        if frequency > 0:
            rate = rospy.Rate(frequency)

        bridge = CvBridge()

        while True:
            img_msg = bridge.cv2_to_imgmsg(
                    img,
                    encoding=encoding
            )
            self.pub.publish(img_msg)

            if frequency == 0:
                break

            rate.sleep()

    def img_stream_subscribe(
            self,
            callback,
            loop=True
    ):
        def _callback(img_msg):
            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(
                    img_msg, 
                    desired_encoding='bgr8'
            )
            callback(cv_img)

        if self.in_bag_name == None:
            self.sub = rospy.Subscriber(
                    self.sub_topic_name, 
                    Image, 
                    callback=_callback,
                    queue_size=self.queue_size
            )
        
            if loop:
                rospy.spin()
        else:
            bag = rosbag.Bag(self.in_bag)

            last_time = None

            for t, msg, topic in bag.read_messages(topic=[self.sub_topic_name]):
                if last_time != None:
                    rospy.sleep(t - last_time)

                _callback(msg)

                last_time = t

########################################################################
# Methods:

def get_args():
    sys.argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser()

    parser.add_argument(
            '-s',
            help="Create subscriber. Default is publisher.",
            action='store_true',
            default=False,
            dest='s'
    )
    parser.add_argument(
            '-b',
            help="The input bag. Will not listen to real time topics.",
            type=str,
            action="store",
            default=None
    )
    parser.add_argument(
            '-t',
            help="The topic name.",
            action='store',
            type=str,
            default="/img_stream"
    )
    parser.add_argument(
            '-n',
            help="The node name.",
            action='store',
            type=str,
            default="img_stream"
    )
    parser.add_argument(
            '-v',
            help='The video file name.',
            action='store',
            type=str,
            default=None
    )

    return parser.parse_args(sys.argv[1:])

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()

    rospy.init_node(args.n)

    if (args.v == None):
        args.v = "/home/fn/Workspace/Uni/2RoboticsAndComputerVision/Project3/videos/2017_06_23_1430 Falen Cigaren mod byen-RgriGiFe-u4.mp4"
    
    ros_video = None

    if args.s:
        def callback(frame):
            cv2.imshow(
                    "frame",
                    frame
            )
            cv2.waitKey(1)

        ros_video = ROSImageStream(
                sub_topic_name=args.t,
                in_bag_name=args.b
        )

        ros_video.img_stream_subscribe(callback)
    else:
        ros_video = ROSImageStream(pub_topic_name=args.t)

        ros_video.publish_from_video(args.v)
