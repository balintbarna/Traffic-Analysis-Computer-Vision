#!/usr/bin/env python2.7

"""
    Module implementing the homography node.

    Change log:
    Created     frnyb       20200416
"""

########################################################################
# Imports:

import sys
import argparse
import json

import rospy
import rospkg

from homography.homography import Homography

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
            default="homography"
    )

    parser.add_argument(
            "-i",
            help="Input image stream.",
            type=str,
            action="store",
            default="/homography_input"
    )

    parser.add_argument(
            "-o",
            help="Output image stream.",
            type=str,
            action="store",
            default="/homography_output"
    )

    return parser.parse_args(sys.argv[1:])

def get_config(package):
    rospack = rospkg.RosPack()

    package_path = rospack.get_path(package)

    config_path = package_path + "/configuration/config.json"

    config = None
    with open(config_path) as f:
        config = json.load(f)

    return config

########################################################################
# Main:

if __name__ == '__main__':
    args = get_args()

    config = get_config("traffic_analysis_from_drones")

    rospy.init_node(args.n)

    h = Homography(
            config["img_coords"],
            config["utm_coords"],
            config["coord_scale_offset"],
            args.i,
            args.o
    )
    h.start()

