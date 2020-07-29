# Motoman experimental

[![Build Status: Travis CI](https://travis-ci.com/ros-industrial/motoman_experimental.svg?branch=kinetic-devel)](https://travis-ci.com/ros-industrial/motoman_experimental)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 3 clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Experimental packages for Motoman manipulators within [ROS-Industrial][].
See the [ROS wiki][] page for more information.


## Contents

This repository contains packages that will be migrated to the [motoman][] repository after they have received sufficient testing.
The contents of these packages are subject to change, without prior notice.
Any available APIs are to be considered unstable and are not guaranteed to be complete and / or functional.

Branch naming follows the ROS distribution they are compatible with.
`-devel` branches may be unstable.
As these are experimental packages, they will not be released through the ROS buildfarm and must be built from sources in a [Catkin workspace][].


## Building

### On newer (or older) versions of ROS

Building the packages on newer (or older) versions of ROS is in most cases possible and supported.
For example: building the packages in this repository on Ubuntu Xenial/ROS Kinetic or Ubuntu Bionic/ROS Melodic systems is supported.
This will require creating a [Catkin workspace][], cloning this repository, installing all required dependencies and finally building the workspace.

### Catkin tools

It is recommended to use [catkin_tools][] instead of the default [catkin][] when building ROS workspaces.
`catkin_tools` provides a number of benefits over regular `catkin_make` and will be used in the instructions below.
All packages can be built using `catkin_make` however: use `catkin_make` in place of `catkin build` where appropriate.

### Building the packages

The following instructions assume that a [Catkin workspace][] has been created at `$HOME/catkin_ws` and that the *source space* is at `$HOME/catkin_ws/src`.
Update paths appropriately if they are different on the build machine.

These instructions build the `kinetic-devel` branch on a ROS Kinetic system:

```bash
# change to the root of the Catkin workspace
$ cd $HOME/catkin_ws

# retrieve the latest development version of motoman. If you'd rather
# use the latest released version, replace 'kinetic-devel' with 'kinetic'
$ git clone -b kinetic-devel \
  https://github.com/ros-industrial/motoman.git \
  src/motoman

# retrieve the latest development version of motoman_experimental.
$ git clone -b kinetic-devel \
  https://github.com/ros-industrial/motoman_experimental.git \
  src/motoman_experimental

# check build dependencies. Note: this may install additional packages,
# depending on the software installed on the machine
$ rosdep update

# be sure to change 'kinetic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro kinetic

# build the workspace (using catkin_tools)
$ catkin build
```

### Activating the workspace

Finally, activate the workspace to get access to the packages just built:

```bash
$ source $HOME/catkin_ws/devel/setup.bash
```

At this point all packages should be usable (ie: `roslaunch` should be able to auto-complete package names starting with `motoman_..`).
In case the workspace contains additional packages (ie: not from this repository), those should also still be available.


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/motoman_experimental
[motoman]: https://github.com/ros-industrial/motoman
[Catkin workspace]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[catkin]: http://wiki.ros.org/catkin
[catkin_tools]: https://catkin-tools.readthedocs.io/en/latest
