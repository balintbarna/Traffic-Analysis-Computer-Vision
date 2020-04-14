#!/usr/bin/env python2.7

"""
    Module with classes MultiTracker and MultiTrackerReceiver for 
    tracking multiple objects visually using several Tracker type objects.
    MultiTrackerReceiver provides functionality for reading the tracked 
    objects information from the topics published to by MultiTracker. 
    Gives full abstraction from the ROS communication underneath.

    Change log: 
    Created     frnyb       20200401

    Rewritten for Python 2.7:
    Shebang, contour finding
                frnyb       20200410
"""

########################################################################
# Imports:

import sys
import argparse
import threading
from copy import copy

import cv2
import numpy as np

import rospy

from ros_video.ros_img_stream import ROSImageStream
from pre_tracker import PreTracker
from shift_tracker import CamshiftTracker

from tracking.msg import Window, Status, Event

from tracked_objects import TrackedObject, TrackedObjects

########################################################################
# Classes:

class MultiTracker:
    def __init__(
            self,
            tracker_type=CamshiftTracker, # MeanshiftTracker, CamshiftTracker
            cnt_min_area=500,
            window_max_width=None,
            lost_ttl_attempts=10,
            max_tracked_objects=30,
            overlap_margin=25,
            pre_tracker=None,
            in_frames_topic="/img_stream",
            out_events_topic="/multi_tracker/global_events",
            out_status_topic="/multi_tracker/status",
            out_window_topic="/multi_tracker/window",
            in_kalman_topic="/multi_tracker/pos",
            out_frames_topic="/multi_tracker/tracked_frames"
    ):
        self.tracker_type = tracker_type
        self.cnt_min_area = cnt_min_area
        self.window_max_width = window_max_width
        self.lost_ttl_attempts = lost_ttl_attempts
        self.max_tracked_objects = max_tracked_objects
        self.overlap_margin = overlap_margin

        if pre_tracker == None:
            self.pre_tracker = PreTracker()#processed_frame_topic="/pre_tracked_frame")
            self.pre_tracker.set_color_range(color_range=((0, 0, 0), (20, 20, 20)))
            self.pre_tracker.set_median_blur(7)
            self.pre_tracker.set_pre_track_callbacks(
                    [self.pre_tracker.pre_track_color_range, self.pre_tracker.pre_track_median_blur]
            )
        else:
            self.pre_tracker = pre_tracker

        self.in_frames_topic = in_frames_topic
        self.out_events_topic = out_events_topic
        self.out_status_topic = out_status_topic
        self.out_window_topic = out_window_topic
        self.in_kalman_topic = in_kalman_topic
        self.out_frames_topic = out_frames_topic

        self.frames_img_stream = ROSImageStream(sub_topic_name=self.in_frames_topic)

        self.kalman_sub = rospy.Subscriber(
                self.in_kalman_topic,
                Window,
                callback=self.kalman_pos_callback,
                queue_size=self.max_tracked_objects
        )

        if self.out_frames_topic != None:
            self.out_frames_pub = ROSImageStream(pub_topic_name=self.out_frames_topic)
        else:
            selt.out_frames_pub = None

        self.tracked_objects = TrackedObjects(
                out_status_topic=self.out_status_topic,
                out_window_topic=self.out_window_topic,
                out_events_topic=self.out_events_topic,
                max_tracked_objects=self.max_tracked_objects,
                max_ttl=self.lost_ttl_attempts
        )

        self.trackers = {} # {"id": tracker_type}

    def start(self):
        self.frames_img_stream.img_stream_subscribe(self.apply_tracking)

    def apply_tracking(
            self,
            frame
    ):
        pre_tracked_frame = self.pre_tracker.pre_track(frame)

        updated_tracked_frame = self.update_current_trackers(pre_tracked_frame)

        self.discover_new_objects(updated_tracked_frame)

        if self.out_frames_pub != None:
            self.publish_tracked_frame(pre_tracked_frame)

    def update_current_trackers(
            self,
            pre_tracked_frame
    ):
        for obj in self.tracked_objects:
            obj.acquire_lock()

            ret, track_window = self.trackers[obj.id].track(pre_tracked_frame)

            if self.window_max_width != None:
                if track_window[2] > self.window_max_width and track_window[3] > self.window_max_width:
                    track_window = (
                            track_window[0],
                            track_window[1],
                            self.window_max_width,
                            self.window_max_width
                    )
                elif track_window[2] > self.window_max_width:
                    track_window = (
                            track_window[0],
                            track_window[1],
                            self.window_max_width,
                            track_window[3]
                    )
                elif track_window[3] > self.window_max_width:
                    track_window = (
                            track_window[0],
                            track_window[1],
                            track_window[2],
                            self.window_max_width
                    )


            #if (
            #        obj.track_window[0] + obj.track_window[2] < 0 or
            #        obj.track_window[0] > pre_tracked_frame.shape[1] or
            #        obj.track_window[1] + obj.track_window[3] < 0 or
            #        obj.track_window[1] > pre_tracked_frame.shape[0]
            #):
            #    pre_tracked_frame = self.handle_lost_object(
            #            obj,
            #            pre_tracked_frame
            #    )

            #    obj.release_lock()

            #    continue

            x_lowerb, x_upperb, y_lowerb, y_upperb = self.get_slices(
                    track_window,
                    pre_tracked_frame
            )

            sub_frame = pre_tracked_frame[
                    x_lowerb:x_upperb,
                    y_lowerb:y_upperb
            ]

            _, cnts, _ = cv2.findContours(
                    sub_frame,
                    cv2.RETR_TREE,
                    cv2.CHAIN_APPROX_SIMPLE
            )
            
            if len(cnts) == 0:
                pre_tracked_frame = self.handle_lost_object(
                        obj,
                        pre_tracked_frame
                )

                obj.release_lock()

                continue

            max_cnt = cnts[0]
            max_cnt_area = cv2.contourArea(max_cnt)

            for cnt in cnts:
                cnt_area = cv2.contourArea(cnt)

                if cnt_area > max_cnt_area:
                    max_cnt_area = cnt_area

            if max_cnt_area < self.cnt_min_area:
                pre_tracked_frame = self.handle_lost_object(
                        obj,
                        pre_tracked_frame
                )

                obj.release_lock()

                continue

            obj.track_window = track_window

            pre_tracked_frame = self.handle_ok_object(
                    obj,
                    pre_tracked_frame
            )

            obj.release_lock()

        return pre_tracked_frame

    def handle_lost_object(
            self,
            obj,
            pre_tracked_frame
    ):
        obj.set_lost()

        x_lowerb, x_upperb, y_lowerb, y_upperb = self.get_slices(
                obj.track_window,
                pre_tracked_frame
        )

        pre_tracked_frame[
            x_lowerb:x_upperb,
            y_lowerb:y_upperb
        ] = np.full(
                pre_tracked_frame[
                    x_lowerb:x_upperb,
                    y_lowerb:y_upperb
                ].shape,
                0
        )

        return pre_tracked_frame

    def handle_ok_object(
            self,
            obj,
            pre_tracked_frame
    ):
        obj.set_ok()

        x_lowerb, x_upperb, y_lowerb, y_upperb = self.get_slices(
                obj.track_window,
                pre_tracked_frame
        )

        pre_tracked_frame[
            x_lowerb:x_upperb,
            y_lowerb:y_upperb
        ] = np.full(
                pre_tracked_frame[
                    x_lowerb:x_upperb,
                    y_lowerb:y_upperb
                ].shape,
                0
        )

        return pre_tracked_frame

    def get_slices(
            self,
            track_window,
            pre_tracked_frame
    ):
        x_lowerb = track_window[1] - self.overlap_margin
        if x_lowerb < 0:
            x_lowerb = 0
        x_upperb = track_window[1] + track_window[3] + self.overlap_margin
        if x_upperb > pre_tracked_frame.shape[0]:
            x_upperb = pre_tracked_frame.shape[0]
        y_lowerb = track_window[0] - self.overlap_margin
        if y_lowerb < 0:
            y_lowerb = 0
        y_upperb = track_window[0] + track_window[2] + self.overlap_margin
        if y_upperb > pre_tracked_frame.shape[1]:
            y_upperb = pre_tracked_frame.shape[1]

        return x_lowerb, x_upperb, y_lowerb, y_upperb

    def discover_new_objects(
            self,
            updated_frame
    ):
        _, cnts, _ = cv2.findContours(
                updated_frame,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE
        ) 

        bounding_rects = []

        for cnt in cnts:
            rect = cv2.boundingRect(cnt)

            if self.window_max_width != None:
                if rect[2] > self.window_max_width and rect[3] > self.window_max_width:
                    rect = (
                            rect[0],
                            rect[1],
                            self.window_max_width,
                            self.window_max_width
                    )
                elif rect[2] > self.window_max_width:
                    rect = (
                            rect[0],
                            rect[1],
                            self.window_max_width,
                            rect[3]
                    )
                elif rect[3] > self.window_max_width:
                    rect = (
                            rect[0],
                            rect[1],
                            rect[2],
                            self.window_max_width
                    )


            overlapping = False

            for b_rect in bounding_rects:
                if not (
                        rect[1] + rect[2] < b_rect[1] or
                        b_rect[1] + b_rect[2] < rect[1] or
                        rect[0] + rect[3] < b_rect[0] or
                        b_rect[0] + b_rect[3] < rect[0]
                ):
                    overlapping = True
                    break

            if overlapping:
                continue

            cnt_area = cv2.contourArea(cnt)

            if cnt_area < self.cnt_min_area:
                continue

            bounding_rects.append(rect)

            key = self.tracked_objects.create(window=rect)

            if key != None:
                self.trackers[key] = self.tracker_type(init_window=rect)

    def kalman_pos_callback(
            self,
            msg
    ):
        obj = self.tracked_objects.get_item(msg.id)

        if obj != None:
            obj.acquire_lock()

            if obj.status == "lost":
                obj.track_window = msg.window

            obj.release_lock()

    def publish_tracked_frame(
            self,
            frame
    ):
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        for obj in self.tracked_objects.tracked_objects:
            obj.acquire_lock()

            if obj.status == "ok":
                frame = cv2.rectangle(
                        frame,
                        (obj.track_window[0], obj.track_window[1]),
                        (obj.track_window[0] + obj.track_window[2], obj.track_window[1] + obj.track_window[3]),
                        (255,0,0),
                        thickness=3
                )
            elif obj.status == "lost":
                frame = cv2.rectangle(
                        frame,
                        (obj.track_window[0], obj.track_window[1]),
                        (obj.track_window[0] + obj.track_window[2], obj.track_window[1] + obj.track_window[3]),
                        (0,0,255),
                        thickness=3
                )

            obj.release_lock()

        self.out_frames_pub.publish_single(frame)

class MultiTrackerReceiver:
    def __init__(
            self,
            max_tracked_objects=30,
            event_callback=None, #callback(key, event)
            status_callback=None, #callback(key, status)
            position_callback=None, #callback(key, pos)
            in_events_topic="/multi_tracker/global_events",
            in_status_topic="/multi_tracker/status",
            in_window_topic="/multi_tracker/window",
            out_kalman_topic="/multi_tracker/pos"
    ):
        self.max_tracked_objects = max_tracked_objects

        self.event_callback = event_callback
        self.status_callback = status_callback
        self.position_callback = position_callback

        self.in_events_topic = in_events_topic
        self.in_status_topic = in_status_topic
        self.in_window_topic = in_window_topic
        self.out_kalman_topic = out_kalman_topic

        self.tracked_objects = TrackedObjects(
                max_tracked_objects=max_tracked_objects,
                locked=True
        )

    def start(
            self,
            loop=True
    ):
        self.events_topic = rospy.Subscriber(
                self.in_events_topic,
                Event,
                callback=self._event_callback,
                queue_size=self.max_tracked_objects
        )

        self.status_topic = rospy.Subscriber(
                self.in_status_topic,
                Status,
                callback=self._status_callback,
                queue_size=self.max_tracked_objects
        )

        self.window_sub = rospy.Subscriber(
                self.in_window_topic,
                Window,
                callback=self._window_callback,
                queue_size=self.max_tracked_objects
        )

        if self.out_kalman_topic != None:
            self.kalman_pub = rospy.Publisher(
                    self.out_kalman_topic,
                    Window,
                    queue_size=self.max_tracked_objects
            )
        else:
            self.kalman_pub = None

        if loop:
            rospy.spin()

    def stop(self):
        self.events_topic.unregister()
        self.events_topic = None

        self.status_topic.unregister()
        self.status_topic = None

        self.window_sub.unregister()
        self.window_sub = None

        if self.out_kalman_topic != None:
            self.kalman_pub.unregister()
            self.kalman_pub = None
        else:
            self.kalman_pub = None

    def _event_callback(
            self,
            msg
    ):
        self.tracked_objects.acquire_lock()

        if msg.event == "+":
            pass
        #    if msg.id in list(self.tracked_objects.keys):
        #        self.tracked_objects.remove(msg.id)

        #    self.tracked_objects.create(key=msg.id)
        elif msg.event == "-":
            if msg.id in list(self.tracked_objects.keys):
                self.tracked_objects.remove(msg.id)

        self.tracked_objects.release_lock()

        if self.event_callback != None:
            self.event_callback(
                    msg.id,
                    msg.event
            )

    def _status_callback(
            self,
            msg
    ):
        self.tracked_objects.acquire_lock()

        if msg.id in list(self.tracked_objects.keys):
            self.tracked_objects.get_item(msg.id).status = msg.status

        self.tracked_objects.release_lock()

        if self.status_callback != None:
            self.status_callback(
                    msg.id,
                    msg.status
            )

    def _window_callback(
            self,
            msg
    ):
        self.tracked_objects.acquire_lock()

        if msg.id in self.tracked_objects.keys:
            pass
            #self.tracked_objects.tracked_objects[self.tracked_objects.keys.index(msg.id)].window = msg.window
        else:
            self.tracked_objects.create(
                    key=msg.id,
                    window=msg.window
            )

        self.tracked_objects.release_lock()

        if self.position_callback != None:
            position = ((msg.window[1] + (msg.window[3] / 2)), msg.window[0] + (msg.window[2] / 2))
            self.position_callback(
                    msg.id,
                    position
            )

    def supply_kalman_estimate(
            self,
            pos,
            id
    ):
        self.tracked_objects.acquire_lock()

        if id in list(self.tracked_objects.keys):
            obj = self.tracked_objects.tracked_objects[self.tracked_objects.keys.index(id)]

            if obj.status == "lost" and obj.track_window != None:
                obj.track_window = ( 
                        int(pos[1] - (obj.track_window[2] / 2)),
                        int(pos[0] - (obj.track_window[3] / 2)),
                        int(obj.track_window[2]),
                        int(obj.track_window[3])
                )

                msg = Window()
                msg.id = id
                msg.window = obj.track_window

                self.kalman_pub.publish(msg)

        self.tracked_objects.release_lock()

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

    return parser.parse_args(sys.argv[1:])

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()

    rospy.init_node(
            args.n,
            disable_signals=True
    )

    mt = MultiTracker(
            cnt_min_area=args.object_min_area,
            lost_ttl_attempts=args.lost_attempts,
            max_tracked_objects=args.max_tracked_objects,
            overlap_margin=args.overlap_margin,
            in_frames_topic=args.i,
            out_frames_topic=args.o,
            out_events_topic=args.events_topic,
            out_window_topic=args.window_topic,
            out_status_topic=args.status_topic,
            in_kalman_topic=args.kalman_topic
    )

    mt.start()
    
