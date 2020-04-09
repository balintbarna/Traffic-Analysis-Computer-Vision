#!/usr/bin/env python3.6

"""
    Module with class RTKalmanFilter for real time Kalman Filtering..

    Change log: 
    Created     frnyb       20200406
"""

########################################################################
# Imports:

import sys
import argparse
import threading

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import rosbag
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Header

import numpy as np

########################################################################
# Main:

class RTKalmanFilter:
    def __init__(
            self,
            dt_prediction=0.05,
            sensor_topic="/sensor",
            sensor_bag=None,
            prediction_topic=None,
            update_topic=None,
            state_est_topic="/rt_kalman_filter/state_est"
    ):
        self.dt_prediction = dt_prediction

        self.sensor_topic = sensor_topic
        self.sensor_bag = sensor_bag
        self.prediction_topic = prediction_topic
        self.update_topic = update_topic
        self.state_est_topic = state_est_topic

        self.sub_sensor = None
        self.bag_sensor = None
        self.pub_prediction = None
        self.pub_update = None
        self.pub_state_est = None
    
        self.filter_lock = threading.Lock()

        self.update_thread = None
        self.prediction_thread = None

    def init_filter(
            self,
            x,
            F,
            H,
            P,
            R,
            Q
    ):
        self.filter = KalmanFilter(
                x.shape[0],
                H.shape[0]
        )

        self.filter.x = x
        self.filter.F = F
        self.filter.H = H
        self.filter.P = P
        self.filter.R = R
        self.filter.Q = Q

    def start(
            self,
            loop=True,
            callbacks=None, # {"prediction/update": {"callback": callback, "args": ()}}
            start_update=True,
            start_prediction=True
    ):
        self.callbacks=callbacks

        self.filter_lock.acquire()

        if self.update_topic != None:
            self.pub_prediction = rospy.Publisher(
                        self.update_topic,
                        Float32MultiArray,
                        queue_size=10
            )

        if self.prediction_topic != None:
            self.pub_prediction = rospy.Publisher(
                        self.prediction_topic,
                        Float32MultiArray,
                        queue_size=10
            )

        if self.state_est_topic != None:
            self.pub_state_est = rospy.Publisher(
                        self.state_est_topic,
                        Float32MultiArray,
                        queue_size=10
            )

        self.filter_lock.release()

        if start_update:
            self.update_thread = threading.Thread(
                    target=self.update_loop, 
                    daemon=True
            )
            self.update_thread.start()
        else:
            self.update_thread = None

        if start_prediction:
            self.prediction_thread = threading.Thread(
                    target=self.predict_loop,
                    daemon=True
            )
            self.prediction_thread.start()
        else:
            self.prediction_thread = None

        if loop:
            try:
                while True:
                    rospy.sleep(0.1)
            except KeyboardInterrupt:
                self.stop()

    def stop(self):
        self.filter_lock.acquire()

        self.filter = None

        if self.pub_prediction != None:
            self.pub_prediction.unregister()
            self.pub_prediction = None
        if self.pub_update != None:
            self.pub_update.unregister()
            self.pub_update = None
        if self.pub_state_est != None:
            self.pub_state_est.unregister()
            self.pub_state_est = None

        self.filter_lock.release()

        self.update_thread = None
        self.prediction_thread = None

    def get_estimate(self):
        self.filter_lock.acquire()

        est = self.filter.x

        self.filter_lock.release()

        return est

    def set_callbacks(
            self,
            callbacks # {"prediction/update": {"callback": callback, "args": ()}}
    ):
        self.filter_lock.acquire()

        self.callbacks = callbacks

        self.filter_lock.release()

    def predict_loop(self):
        self.t_predict = rospy.get_rostime() + rospy.rostime.Duration(secs=self.dt_prediction)

        while True:
            rospy.sleep(self.t_predict - rospy.get_rostime())

            self.t_predict = self.t_predict + rospy.rostime.Duration(secs=self.dt_prediction)

            self.filter_lock.acquire()

            if self.filter == None:
                self.filter_lock.release()
                break

            self.filter.predict()

            state_msg = Float32MultiArray()
            state_msg.data=list(self.filter.x)

            if self.pub_prediction != None:
                self.pub_prediction.publish(state_msg)
            if self.pub_state_est != None:
                self.pub_state_est.publish(state_msg)

            if self.callbacks != None:
                if 'prediction' in self.callbacks.keys():
                    self.callbacks['prediction']["callback"](
                            state_msg.data,
                            self.callbacks["prediction"]["arg"]
                    )

            self.filter_lock.release()

    def update_loop(self):
        if self.update_topic != None:
            self.pub_update = rospy.Publisher(
                    self.update_topic,
                    Float32MultiArray,
                    queue_size=1
            )

        if self.sensor_bag != None:
            self.bag_sensor = rosbag.Bag(self.sensor_bag)

            current_time = None

            for topic, msg, t in self.bag_sensor.read_messages():
                if current_time == None:
                    current_time = t.to_sec()

                rospy.sleep(t.to_sec() - current_time)
                
                stop = self.update_step(
                        msg,
                        True
                )

                if stop:
                    break

            self.stop()
        else:
            if self.filter.H.shape[0] == 1:
                self.sub_sensor = rospy.Subscriber(
                        self.sensor_topic,
                        Float32,
                        self.update_step,
                        queue_size=1
                )
            else:
                self.sub_sensor = rospy.Subscriber(
                        self.sensor_topic,
                        Float32MultiArray,
                        self.update_step,
                        queue_size=1
                )

            while True:
                rospy.sleep(0.01)

                self.filter_lock.acquire()

                if self.filter == None:
                    self.sub_sensor.unregister()
                    
                    self.filter_lock.release()

                    break

                self.filter_lock.release()

        if self.sensor_bag != None:
            self.sensor_bag = None

    def update_step(
            self, 
            sensor_msg,
            check_stop=False
    ):
        self.filter_lock.acquire()

        if self.filter != None:
            try:
                if self.filter.H.shape[0] == 1:
                    self.filter.update(np.array([sensor_msg.data]))
                else:
                    self.filter.update(np.array(sensor_msg.data))
            except AttributeError:
                self.filter.update(np.array([[sensor_msg[0]],[sensor_msg[1]]]))


            state_msg = Float32MultiArray()
            state_msg.data = list(self.filter.x)

            if self.pub_update != None:
                self.pub_update.publish(state_msg)
            if self.pub_state_est != None:
                self.pub_state_est.publish(state_msg)
            
            if self.callbacks != None:
                if 'update' in self.callbacks.keys():
                    self.callbacks['update']["callback"](
                            state_msg.data, 
                            self.callbacks["update"]["arg"]
                    )

        if check_stop:
            res = self.filter == None

            self.filter_lock.release()

            return res
        else:
            self.filter_lock.release()

def get_args():
    sys.argv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-n",
        help="The node name.",
        type=str,
        action="store",
        default="rt_kalman_filter"
    )

    parser.add_argument(
            "-s",
            help="Delta seconds between predictions.",
            type=float,
            action="store",
            default=0.05
    )

    parser.add_argument(
            "--intopic",
            help="The sensor signal topic.",
            type=str,
            action="store",
            default="/measurement"
    )

    parser.add_argument(
            "--inbag",
            help="The bag to load sensor data from.",
            type=str,
            action="store",
            default=None
    )

    parser.add_argument(
            "--outtopic",
            help="The topic on which to broadcast all state estimates.",
            type=str,
            action="store",
            default="state_est"
    )

    parser.add_argument(
            "--pred-topic",
            help="Topic on which to broadcast prediction output.",
            type=str,
            action="store",
            default=None
    )

    parser.add_argument(
        "--update-topic",
        help="Topic on which to broadcast prediction output.",
        type=str,
        action="store",
        default=None
    )

    parser.add_argument(
            "--second-order",
            help="Will use a default second order system.",
            action="store_true",
            default=False
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

    rtkf = RTKalmanFilter(
            dt_prediction=args.s,
            sensor_topic=args.intopic,
            sensor_bag=args.inbag,
            prediction_topic=args.pred_topic,
            update_topic=args.update_topic,
            state_est_topic=args.outtopic
    )

    x = F = H = P = R = Q = None

    if args.second_order:
        x = np.array([[2.], [0.], [0.]])
        F = np.array([[1., args.s, (args.s**2)/2], [0., 1, args.s], [0, 0, 1]])
        H = np.array([[1., 0., 1.]])
        P = np.array([[10. ,0., 0.], [0., 10., 0.], [0., 0., 10.]])
        R = np.array([[1.5]])
        Q = Q_discrete_white_noise(dim=3, var=0.13)
    else:
        x = np.array([[2.], [0.]])
        F = np.array([[1., args.s], [0., 1.]])
        H = np.array([[1., 0.]])
        P = np.array([[10. ,0.], [0., 10.]])
        R = np.array([[1.5]])
        Q = Q_discrete_white_noise(dim=2, var=0.13)

    rtkf.init_filter(
            x,
            F,
            H,
            P,
            R,
            Q
    )

    rtkf.start()

    #rospy.sleep(10)
    #rtkf.stop()



