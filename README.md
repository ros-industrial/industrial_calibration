industrial_calibration
======================

:warning: EXPERIMENTAL KINETIC-DEVEL branch :warning:
===

## ROS Distro Support

|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo-devel`](https://github.com/ros-industrial/industrial_calibration/tree/indigo-devel) | [`indigo-devel`](https://github.com/ros-industrial/industrial_calibration/tree/indigo-devel) | [`kinetic-devel`](https://github.com/ros-industrial/industrial_calibration/tree/kinetic-devel) |
| Status  |  supported | supported |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=industrial_calibration) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=industrial_calibration) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=industrial_calibration) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.org/ros-industrial/industrial_calibration.svg?branch=kinetic-devel)](https://travis-ci.org/ros-industrial/industrial_calibration)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Jade Source | Jade Debian |  Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| industrial_calibration | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__industrial_calibration__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__industrial_calibration__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__industrial_calibration__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__industrial_calibration__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__industrial_calibration__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__industrial_calibration__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__industrial_calibration__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__industrial_calibration__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__industrial_calibration__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__industrial_calibration__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__industrial_calibration__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__industrial_calibration__ubuntu_xenial_amd64__binary/) |


Contains libraries/algorithms for calibration industrial systems

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
