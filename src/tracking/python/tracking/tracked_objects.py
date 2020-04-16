#!/usr/bin/env python2.7

"""
    Module with classes TrackedObject and TrackedObjects.

    Change log: 
    Created     frnyb       20200406

    Rewritten for Python 2.7:
    Shebang and copy
                frnyb       20200410
"""

########################################################################
# Imports:

import threading
from copy import copy

import numpy as np

import rospy

from tracking.msg import Window, Status, Event

########################################################################
# Classes:

class TrackedObject:
    def __init__(
            self,
            _id=None,
            track_window=None,
            status=None,
            max_ttl=None,
            status_pub=None,
            window_pub=None,
            locked=True
    ):
        self.id = _id
        self.track_window = track_window
        self.last_window = None
        self.status = status
        self.max_ttl = max_ttl
        self.ttl = self.max_ttl

        self.status_pub = status_pub
        self.window_pub = window_pub

        if locked:
            self.lock = threading.Lock()
        else:
            self.lock = None

    def acquire_lock(
            self,
            _timeout=None
    ):
        if self.lock == None:
            return True

        if _timeout == None or _timeout < 0:
            return self.lock.acquire()
        elif _timeout == 0:
            return self.lock.acquire(blocking=False)
        else:
            return self.lock.acquire(timeout=_timeout)

    def release_lock(self):
        if self.lock != None:
            self.lock.release()

    def is_locked(self):
        if self.lock == None:
            return False
        else:
            return self.lock.locked()

    def set_ok(self):
        if self.ttl != None:
            self.ttl = self.max_ttl

        if self.status != "ok" and self.status_pub != None:
            msg = Status()
            msg.id = self.id
            msg.status = "ok"
            self.status_pub.publish(msg)

        if self.window_pub != None:
            msg = Window()
            msg.id = self.id
            msg.window = self.track_window
            self.window_pub.publish(msg)

        self.status = "ok"

    def set_lost(self):
        if self.ttl != None:
            self.ttl -= 1

            if self.ttl <= 0:
                self.set_removed()
                return
            elif self.status != "lost" and self.status_pub != None:
                msg = Status()
                msg.id = self.id
                msg.status = "lost"
                self.status_pub.publish(msg)

        self.status = "lost"
                
    def set_removed(self):
        self.status = "removed"

        if self.status_pub != None:
            msg = Status()
            msg.id = self.id
            msg.status = "removed"
            self.status_pub.publish(msg)

    def supply_window(
            self,
            window
    ):
        #if self.track_window != None and self.status == "lost":
        self.last_window = self.track_window
        self.track_window = window
            #self.track_window = (
            #        pos[1] + (self.track_window[2] / 2),
            #        pos[0] + (self.track_window[3] / 2),
            #        self.track_window[2],
            #        self.track_window[3]
            #)

class TrackedObjects:
    def __init__(
            self,
            out_status_topic=None,
            out_window_topic=None,
            out_events_topic=None,
            max_tracked_objects=10,
            max_ttl=None,
            locked=False
    ):
        self.out_status_topic = out_status_topic
        self.out_window_topic = out_window_topic
        self.out_events_topic = out_events_topic

        self.max_tracked_objects = max_tracked_objects
        self.max_ttl = max_ttl

        if self.out_status_topic != None:
            self.status_pub = rospy.Publisher(
                    self.out_status_topic,
                    Status,
                    queue_size=self.max_tracked_objects
            )
        else:
            self.status_pub = None

        if self.out_window_topic != None:
            self.window_pub = rospy.Publisher(
                    self.out_window_topic,
                    Window,
                    queue_size=self.max_tracked_objects
            )
        else:
            self.window_pub = None

        if self.out_events_topic != None:
            self.pub_events = rospy.Publisher(
                    self.out_events_topic,
                    Event,
                    queue_size=self.max_tracked_objects
            )
        else:
            self.pub_events = None

        self.tracked_objects = []
        self.keys = []

        if locked:
            self.lock = threading.Lock()
        else:
            self.lock = True

    def __iter__(self):
        self.remaining_keys = copy(self.keys)

        return self

    def next(self):
        if len(self.remaining_keys) == 0:
            raise StopIteration

        index = 0
        key = self.remaining_keys[index]
        obj = self.get_item(key)

        while True:
            if obj.status == "removed":
                self.remaining_keys.remove(key)
                self.remove(key)

                if self.pub_events != None:
                    msg = Event()
                    msg.id = key
                    msg.event = "-"
                    self.pub_events.publish(msg)

            elif not obj.is_locked():
                break

            if len(self.remaining_keys) == 0:
                raise StopIteration

            index = (index + 1) % len(self.remaining_keys)
            key = self.remaining_keys[index]
            obj = self.get_item(key)

        self.remaining_keys.pop(index)

        return obj

    def __len__(self):
        return len(self.tracked_objects)

    def get_item(
            self,
            key
    ):
        if key in self.keys:
            return self.tracked_objects[self.keys.index(key)]
        else:
            return None

    def acquire_lock(
            self,
            _timeout=None
    ):
        if self.lock == None:
            return True

        if _timeout == None or _timeout < 0:
            return self.lock.acquire()
        elif _timeout == 0:
            return self.lock.acquire(blocking=False)
        else:
            return self.lock.acquire(timeout=_timeout)

    def release_lock(self):
        if self.lock != None:
            self.lock.release()

    def create(
            self,
            window=None,
            status="ok",
            key=None
    ):
        if len(self.tracked_objects) < self.max_tracked_objects:
            if key == None:
                key = 0
                while key in self.keys:
                    key += 1
            elif key in self.keys:
                raise Exception("Key already exists")
                
            obj = TrackedObject(
                    _id=key,
                    track_window=window,
                    status="ok",
                    max_ttl = self.max_ttl,
                    status_pub=self.status_pub,
                    window_pub=self.window_pub
            )

            self.tracked_objects.append(obj)
            self.keys.append(key)

            if self.pub_events != None:
                msg = Event()
                msg.id = key
                msg.event = "+"
                self.pub_events.publish(msg)

            return key
        else:
            return None

    def remove(
            self,
            key
    ):
        if key in self.keys:
            self.tracked_objects.pop(self.keys.index(key))
            self.keys.remove(key)

