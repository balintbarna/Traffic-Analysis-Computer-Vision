#!/usr/bin/env python3.6

"""
    Module with base class Tracker.

    Change log: 
    Created     frnyb       20200404
"""

########################################################################
# Imports:

import sys
import argparse
from abc import ABC, abstractmethod

import cv2
import numpy as np

import rospy

########################################################################
# Classes:

class Tracker(ABC):
    def __init__(
            self,
            init_window=(0,0,1,1)
    ):
        pass

    def track(
            self,
            pre_tracked_frame
    ):
        pass
