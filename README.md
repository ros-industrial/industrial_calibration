industrial_calibration
======================

:warning: EXPERIMENTAL KINETIC-DEVEL branch :warning:
===

Contains libraries/algorithms for calibration industrial systems

# Travis CI

[![Travis-CI](https://api.travis-ci.org/ros-industrial/industrial_calibration.svg?branch=indigo-devel)](https://travis-ci.org/ros-industrial/industrial_calibration/branches)

# Requires
# install ceres-solver(Note, there might be a .deb that works)
#    follow instructions on http://ceres-solver.org/installation.html#linux

# install openni2
sudo apt-get install ros-kinetic-openni2-camera
sudo apt-get install ros-kinetic-openni2-launch

# install moveit
sudo apt-get install ros-kinetic-moveit

# Examples
## Single Basler on a rail
```
roslaunch robocyl_ical.launch
roslaunch robo_cylinder.launch
rosservice call /RobocylCalService "allowable_cost_per_observation: 0.25"

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
