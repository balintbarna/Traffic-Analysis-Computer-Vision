#!/usr/bin/env python2.7

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['kalman_filter'],
        package_dir={'': 'python'}
)

setup(**d)
