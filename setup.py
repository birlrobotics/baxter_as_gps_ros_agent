#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'baxter_as_gps_ros_agent',
    ],
    package_dir={'': 'src'}
)

setup(**setup_args)
