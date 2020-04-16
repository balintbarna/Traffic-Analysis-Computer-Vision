#!/usr/bin/env python2.7

"""
    Module for the kalman tracker.
    
    Change log: 
    Created     frnyb       20200410
"""

########################################################################
# Imports:

import sys
import argparse
import json
import threading
from math import sqrt

import cv2
import numpy as np

import rospy
import rospkg

from tracking.kalman_multi_tracker_receiver import KalmanMultiTrackerReceiver
from ros_video.ros_img_stream import ROSImageStream

########################################################################
# Classes:

class KalmanCarTracker(KalmanMultiTrackerReceiver):
    def __init__(
            self,
            max_tracked_objects=30,
            dt_prediction=0.05,
            gsd=1,
            prediction_topic=None,
            update_topic=None,
            state_est_topic="/kalman_multi_tracker/state_est",
            in_events_topic="/multi_tracker/global_events",
            in_status_topic="/multi_tracker/status",
            in_window_topic="/multi_tracker/window",
            out_kalman_topic="/multi_tracker/pos",
            in_frame_topic="/homography_output",
            out_frame_topic="/out"
    ):
        KalmanMultiTrackerReceiver.__init__(
                self,
                max_tracked_objects,
                dt_prediction,
                prediction_topic,
                update_topic,
                state_est_topic,
                in_events_topic,
                in_status_topic,
                in_window_topic,
                out_kalman_topic
        )

        self.gsd = gsd

        self.frames_sub = ROSImageStream(sub_topic_name=in_frame_topic)
        self.frames_pub = ROSImageStream(pub_topic_name=out_frame_topic)

        self.cars = {}
        self.cars_lock = threading.Lock()

        self.last_id = 0

    def start(self):
        KalmanMultiTrackerReceiver.start(
                self,
                loop=False
        )
        self.frames_sub.img_stream_subscribe(
                callback=self.frame_callback,
                loop=False
        )
        KalmanMultiTrackerReceiver.start(
                self,
                loop=False
        )

        rate = rospy.Rate(5)

        while True:
            rate.sleep()

            self.update_cars()

    def frame_callback(
            self,
            frame
    ):
        self.kalman_filters_lock.acquire()

        for key in self.kalman_filters.keys():
            #print(np.array(self.kalman_filters[key].filter.x).flatten())
            if key not in self.cars.keys():
                continue

            if self.cars[key]["CurrX"] == None or self.cars[key]["MeasurementsN"] < 15:
                continue

            vel_dist = sqrt((self.cars[key]["AvgVelX"] - self.cars[key]["CurrVelX"])**2 + (self.cars[key]["AvgVelY"] - self.cars[key]["CurrVelY"])**2)

            if vel_dist > 4000 / self.cars[key]["MeasurementsN"]:
                continue

            pt1 = np.array(self.kalman_filters[key].filter.x).flatten().astype(np.int16)[:2]
            pt2 = pt1 + 50
            #frame = cv2.rectangle(
            #        frame,
            #        (pt1[0], pt1[1]),
            #        (pt2[0],pt2[1]),
            #        (255, 0, 0)
            #)

            frame = self.write_car(
                    frame,
                    self.cars[key]
            )

        self.frames_pub.publish_single(frame)

        self.kalman_filters_lock.release()

    def write_car(
            self,
            frame,
            car
    ):
        text = str(car["UniqId"]) + ", " + str(int(car["CurrX"] * self.gsd)) + ", " + str(int(car["CurrY"] * self.gsd)) + ", " + str(int(car["CurrVelX"] * self.gsd)) + ", " + str(int(car["CurrVelY"] * self.gsd))
        frame = cv2.putText(
                frame,
                text,
                (int(car["CurrX"]),int(car["CurrY"])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255,0,0)
        )

        return frame

    def update_cars(self):
        self.kalman_filters_lock.acquire()
        self.cars_lock.acquire()

        for key in self.cars.keys():
            if key in self.kalman_filters.keys():
                state = np.array(self.kalman_filters[key].filter.x).flatten()

                self.cars[key]["CurrX"] = state[0]
                self.cars[key]["CurrY"] = state[1]
                self.cars[key]["CurrVelX"] = state[2]
                self.cars[key]["CurrVelY"] = state[3]

                if self.cars[key]["AvgVelX"] == None:
                    self.cars[key]["AvgVelX"] = state[2]
                    self.cars[key]["AvgVelY"] = state[3]
                else:
                    self.cars[key]["AvgVelX"] *= self.cars[key]["MeasurementsN"]
                    self.cars[key]["AvgVelX"] += state[2]
                    self.cars[key]["AvgVelX"] /= self.cars[key]["MeasurementsN"] + 1
                    self.cars[key]["AvgVelY"] *= self.cars[key]["MeasurementsN"]
                    self.cars[key]["AvgVelY"] += state[3]
                    self.cars[key]["AvgVelY"] /= self.cars[key]["MeasurementsN"] + 1

                self.cars[key]["MeasurementsN"] += 1

        self.cars_lock.release()
        self.kalman_filters_lock.release()

    def event_callback(
            self,
            key,
            event
    ):
        self.cars_lock.acquire()

        if key in self.cars.keys():
            self.remove_car(key)

        if event == '+':
            self.create_car(key)

        self.cars_lock.release()

        KalmanMultiTrackerReceiver.event_callback(
                self,
                key,
                event
        )

    def create_car(
            self,
            key
    ):
        self.cars[key] = {
                "UniqId": self.last_id,
                "CurrX": None,
                "CurrY": None,
                "CurrVelX": None,
                "CurrVelY": None,
                "AvgVelX": None,
                "AvgVelY": None,
                "MeasurementsN": 0
        }

        self.last_id += 1

    def remove_car(
            self,
            key
    ):
        if key in self.cars.keys():
            del self.cars[key]

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
            default="kalman_multi_tracker"
    )

    parser.add_argument(
            "-o",
            help="The output Kalman estimated position topic.",
            type=str,
            action="store",
            default="/kalman_multi_tracker/state_est"
    )

    parser.add_argument(
            "--prediction-topic",
            help="The output prediction positions topic.",
            type=str,
            action="store",
            default=None
    )

    parser.add_argument(
            "--update-topic",
            help="The output update positions topic.",
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

    return parser.parse_args(sys.argv[1:])

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()
    
    pack = rospkg.RosPack()

    path = pack.get_path('traffic_analysis_from_drones') + "/configuration/config.json"

    config = None
    with open(path) as f:
        config = json.load(f)

    rospy.init_node(
            args.n,
            disable_signals=True
    )

    kmtr = KalmanCarTracker( 
            max_tracked_objects=config["max_tracked_objects"],
            dt_prediction=config["dt_prediction"],
            gsd=config["gsd"],
            prediction_topic=args.prediction_topic,
            update_topic=args.update_topic,
            state_est_topic=args.o,
            in_events_topic=args.events_topic,
            in_status_topic=args.status_topic,
            in_window_topic=args.window_topic,
            out_kalman_topic=args.kalman_topic
    )

    kmtr.start()

