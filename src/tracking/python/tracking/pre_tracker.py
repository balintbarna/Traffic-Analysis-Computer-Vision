#!/usr/bin/python3.6

"""
    Module with class PreTracker for preprocessing for visual tracking
    pipeline.

    Change log: 
    Created     frnyb       20200330
"""

########################################################################
# Imports:

import sys
import argparse

import rospy
import rosbag
import cv_bridge

import cv2
import numpy as np

from ros_video.ros_img_stream import ROSImageStream

########################################################################
# Classes:

class PreTracker:
    def __init__(
            self,
            in_topic=None,
            in_bag=None,
            processed_frame_topic=None
    ):
        self.pre_track_callbacks = []

        if in_topic != None:
            self.stream_frame = ROSImageStream(
                    sub_topic_name=in_topic,
                    in_bag_name=in_bag
            )

        if processed_frame_topic != None:
            self.stream_processed_frame = ROSImageStream(pub_topic_name=processed_frame_topic)
        else:
            self.stream_processed_frame = None

    def start(self):
        def callback(frame):
            proc_frame = self.pre_track(frame)

        self.stream_frame.img_stream_subscribe(callback)

    def set_pre_track_callbacks(
            self,
            pre_track_callbacks
    ):
        self.pre_track_callbacks = pre_track_callbacks

    def set_color_range(
            self,
            color_range=((0,0,0),(255,255,255))
    ):
        self.color_range = color_range

    def set_background_subtract(self):
        self.background_subtractor = cv2.createBackgroundSubtractorKNN()

    def set_median_blur(
            self,
            ksize
    ):
        self.ksize = ksize

    def pre_track_median_blur(
            self,
            frame
    ):
        return cv2.medianBlur(
                frame,
                self.ksize
        )

    def pre_track_color_range(
            self,
            frame
    ):
        return cv2.inRange(
                frame,
                self.color_range[0],
                self.color_range[1]
        )

    def pre_track_background_subtract(
            self,
            frame
    ):
        return self.background_subtractor.apply(frame)

    def pre_track(
            self,
            frame
    ):
        for callback in self.pre_track_callbacks:
            frame = callback(frame)

        if self.stream_processed_frame != None:
            self.stream_processed_frame.publish_single(
                    frame,
                    encoding="mono8"
            )

        return frame

########################################################################
# Methods:

def get_args():
    sys.argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser()

    parser.add_argument(
            "-n",
            help="The node name.",
            type=str,
            action="store",
            default="pre_tracker"
    )
    parser.add_argument(
            "-i",
            help="The input topic.",
            type=str,
            action="store",
            default="/frame_from_video"
    )
    parser.add_argument(
            "-b",
            help="The input bag. Will not listen to real time topic.",
            type=str,
            action="store",
            default=None
    )
    parser.add_argument(
            "-o",
            help="The output image topic.",
            type=str,
            action="store",
            default="/pre_track"
    )

    return parser.parse_args(sys.argv[1:])

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()

    rospy.init_node(args.n)

    pt = PreTracker(
            in_topic=args.i,
            in_bag=args.b,
            processed_frame_topic=args.o
    )

    pt.set_color_range(color_range=((0, 0, 0),(10, 10, 10)))
    pt.set_background_subtract()
    pt.set_median_blur(13)
    pt.set_pre_track_callbacks([pt.pre_track_median_blur, pt.pre_track_background_subtract, pt.pre_track_median_blur])

    pt.start()
