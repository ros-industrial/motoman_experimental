#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['motoman_package_generator'],
    scripts=['src/motoman_package_generator'],
    package_dir={'': 'src'}
)

setup(**d)

