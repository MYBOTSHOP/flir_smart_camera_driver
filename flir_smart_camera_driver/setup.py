#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['flir_smart_camera_driver'],
    package_dir={'': 'src'},
)

setup(**setup_args)
