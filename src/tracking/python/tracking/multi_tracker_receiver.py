#!/usr/bin/env python2.7

"""
    Module with class MultiTrackerReceiver for interfacing with a 
    MultiTracker node. Can be used as library utility. Non executable.

    Change log: 
    Created     frnyb       20200415
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
