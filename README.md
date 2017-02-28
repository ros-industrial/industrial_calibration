industrial_calibration
======================

:warning: EXPERIMENTAL KINETIC-DEVEL branch :warning:
===

Contains libraries/algorithms for calibration industrial systems

# Travis CI

[![Travis-CI](https://api.travis-ci.org/ros-industrial/industrial_calibration.svg?branch=indigo-devel)](https://travis-ci.org/ros-industrial/industrial_calibration/branches) 

# Build
Requires [wstool](http://wiki.ros.org/wstool)
```
mkdir -p cal_ws/src
cd cal_ws/src
git clone -b kinetic https://github.com/ros-industrial/industrial_calibration.git
wstool merge industrial_calibration/industrial_calibration.rosinstall
wstool update
cd ..
catkin build
```
