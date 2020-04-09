#!/usr/bin/env python3.6

"""
    Module with classes MeanshiftTracker and CamshiftTracker.

    Change log:
    Created     frnyb       20200331
"""

########################################################################
# Imports:

import sys
import argparse

import cv2
import numpy as np

import rospy

from ros_video.ros_img_stream import ROSImageStream
from pre_tracker import PreTracker
from tracker import Tracker

########################################################################
# Classes:

class MeanshiftTracker(Tracker):
    def __init__(
            self,
            init_window=(0,0,1,1)
    ):
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        self.track_window = init_window

    def track(
            self,
            pre_tracked_frame
    ):
        ret, self.track_window = cv2.meanShift(
                pre_tracked_frame,
                self.track_window,
                self.term_crit
        )

        return ret, self.track_window

class CamshiftTracker(Tracker):
    def __init__(
            self,
            init_window=(0,0,1,1)
    ):
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        self.track_window = init_window

    def track(
            self,
            pre_tracked_frame
    ):
        ret, self.track_window = cv2.CamShift(
                pre_tracked_frame,
                self.track_window,
                self.term_crit
        )

        return ret, self.track_window

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
            default="shift_tracker"
    )
    parser.add_argument(
            "-i",
            help="The input frame stream topic name.",
            type=str,
            action="store",
            default="/img_stream"
    )
    parser.add_argument(
            "-o",
            help="The output tracked frame topic name.",
            type=str,
            action="store",
            default="/tracked_frame"
    )

    return parser.parse_args(sys.argv[1:])

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()

    rospy.init_node(args.n)

    class TrackerExample:
        def __init__(self):
            self.t = None
            self.last_frame = None
            self.track_window = None
            self.s = ROSImageStream(sub_topic_name=args.i)

            self.p = ROSImageStream(pub_topic_name=args.o)

            self.pre_tracker = PreTracker()
            self.pre_tracker.set_color_range(((0,0,0),(10,10,10)))
            self.pre_tracker.set_pre_track_callbacks([self.pre_tracker.pre_track_color_range])
        
        def start(self):
            def callback(frame):
                if self.t == None:
                    seg = self.pre_tracker.pre_track(frame)
                    cnts, _ = cv2.findContours(
                            seg,
                            cv2.RETR_TREE,
                            cv2.CHAIN_APPROX_SIMPLE
                    )
                    max_cnt = max(cnts, key=lambda c: cv2.contourArea(c))

                    rectangle = cv2.boundingRect(max_cnt)

                    self.t = CamshiftTracker(init_window=rectangle)

                    r = cv2.rectangle(frame, (rectangle[0],rectangle[1]), (rectangle[2], rectangle[3]), (255, 0, 0),2)

                    self.p.publish_single(r)

                self.last_frame = frame
                pre_tracked_frame = self.pre_tracker.pre_track(frame)
                ret, self.track_window = self.t.track(pre_tracked_frame)

                rectangle = cv2.rectangle(frame, (self.track_window[0], self.track_window[1]), (self.track_window[0] + self.track_window[2], self.track_window[1] + self.track_window[3]), (255,0,0), 2)

                self.p.publish_single(rectangle)
            self.s.img_stream_subscribe(callback)

    ex = TrackerExample() 
    ex.start()
