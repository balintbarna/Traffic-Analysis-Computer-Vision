#!/usr/bin/env python2.7

"""
    Module with base class Tracker.

    Change log: 
    Created     frnyb       20200404

    Rewritten to Python 2.7:
    Shebang changed and Tracker class not inheritting from ABC (Abstract Base Class).
                frnyb       20200410
"""

########################################################################
# Imports:

import sys
import argparse

import cv2
import numpy as np

import rospy

########################################################################
# Classes:

class Tracker():
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
