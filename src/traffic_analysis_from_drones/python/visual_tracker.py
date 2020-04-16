#!/usr/bin/env python2.7

"""
    Module containing the visual tracker node. 
    Utilizes the tracking package.

    Change log:
    Created     frnyb       20200410
    
    Rewritten to include homography as part of pipeline.
                frnyb       20200414
"""

########################################################################
# Imports:

import sys
import argparse
import json
import threading
from copy import copy

import cv2
import numpy as np

import rospy
import rospkg

from tracking.multi_tracker import MultiTracker
from tracking.pre_tracker import PreTracker

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
            default="multi_tracker"
    )

    parser.add_argument(
            "-i",
            help="Input image stream to track.",
            type=str,
            action="store",
            default="/img_stream"
    )

    parser.add_argument(
            "-o",
            help="The output image stream to of the marked tracked objects.",
            type=str,
            action="store",
            default="/multi_tracker/tracked_frames"
    )

    parser.add_argument(
            "--pre-tracked-frames-topic",
            help="Topic to broadcast pre-tracked (pre-processed) frames.",
            type=str,
            action="store",
            default=None
    )

    parser.add_argument(
            "--events-topic",
            help="Topic to communicate global events over.",
            type=str,
            action="store",
            default="/multi_tracker/global_events"
    )

    parser.add_argument(
            "--status-topic",
            help="The topic to communicate tracked objects status over.",
            type=str,
            action="store",
            default="/multi_tracker/status"
    )

    parser.add_argument(
            "--window-topic",
            help="The topic to communicate tracking windows over.",
            type=str,
            action="store",
            default="/multi_tracker/window"
    )

    parser.add_argument(
            "--kalman-topic",
            help="The topic on which to feed back Kalman filter positions when objects are lost.",
            type=str,
            action="store",
            default="/multi_tracker/pos"
    )

    parser.add_argument(
            "--object-min-area",
            help="The minimum area in pixels of an object contour to be tracked.",
            type=int,
            action="store",
            default=500
    )

    parser.add_argument(
            "--lost-attempts",
            help="Number of cycles to wait for a lost object before removing.",
            type=int,
            action="store",
            default=10
    )

    parser.add_argument(
            "--max-tracked-objects",
            help="Maximum number of simoultaneously tracked objects.",
            type=int,
            action="store",
            default=50
    )

    parser.add_argument(
            "--overlap-margin",
            help="Margin around a tracked object in pixels.",
            type=int,
            action="store",
            default=25
    )

    parser.add_argument(
            "--crop-left",
            help="Number of pixels to crop in the right side of the image.",
            type=int,
            action="store",
            default=300
    )

    parser.add_argument(
            "--crop-right",
            help="Number of pixels to crop in the left side of the image.",
            type=int,
            action="store",
            default=300
    )

    parser.add_argument(
            "--crop-top",
            help="Number of pixels to crop in the top of the image.",
            type=int,
            action="store",
            default=300
    )

    parser.add_argument(
            "--crop-bottom",
            help="Number of pixels to crop in the top of the image.",
            type=int,
            action="store",
            default=100
    )

    parser.add_argument(
            "--blur-ksize",
            help="Median blur k size.",
            type=int,
            action="store",
            default=5
    )

    parser.add_argument(
            "--window-max-width",
            help="Maximum side length of track window.",
            type=int,
            action="store",
            default=35
    )

    parser.add_argument(
            "--mask-path",
            help="window-max-widthth to image mask to apply during pre processing.",
            type=str,
            action="store",
            default=None
    )

    return parser.parse_args(sys.argv[1:])

def get_config(package):
    rospack = rospkg.RosPack()

    package_path = rospack.get_path(package)

    config_path = package_path + "/configuration/config.json"

    config = None
    with open(config_path) as f:
        config = json.load(f)

    return package_path, config

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()

    package_path, config = get_config("traffic_analysis_from_drones")

    rospy.init_node(
            args.n,
            disable_signals=True
    )

    pt = PreTracker(processed_frame_topic=args.pre_tracked_frames_topic)

    pt.set_background_subtract()

    er_kernel = np.ones((config["erosion_kernel"],config["erosion_kernel"]),np.uint8)

    def erosion(frame):
        return cv2.erode(frame, er_kernel)

    di_kernel = np.ones((config["dilation_kernel"],config["dilation_kernel"]),np.uint8)

    def dilation(frame):
        return cv2.dilate(frame, di_kernel)

    mask = cv2.imread(package_path + config["mask_path"], 0)

    def apply_mask(frame):
        return cv2.bitwise_and(
                frame,
                frame,
                mask=mask
        )

    def thresh(frame):
        ret, frame = cv2.threshold(frame, config["threshold"], 255, cv2.THRESH_BINARY)

        return frame

    pt.set_pre_track_callbacks([apply_mask, pt.pre_track_background_subtract, erosion, thresh, dilation])

    mt = MultiTracker(
            cnt_min_area=config["object_min_area"],
            window_max_width=config["window_max_width"],
            lost_ttl_attempts=config["lost_attempts"],
            max_tracked_objects=config["max_tracked_objects"],
            overlap_margin=config["overlap_margin"],
            allowed_directions=['w','e'],
            pre_tracker=pt,
            in_frames_topic=args.i,
            out_frames_topic=args.o,
            out_events_topic=args.events_topic,
            out_window_topic=args.window_topic,
            out_status_topic=args.status_topic,
            in_kalman_topic=args.kalman_topic
    )

    mt.start()
    

