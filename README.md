industrial_calibration
======================

:warning: EXPERIMENTAL KINETIC-DEVEL branch :warning:
===

Contains libraries/algorithms for calibration industrial systems.

extrinsic_cal calibration nodes:
service_node   -- peforms quiet a number of calbirations using very complex yaml files
range_excal    -- performs extrinsic calibration of a 3D camera by finding the target points in the image and the xyz values from the depth image.
wrist_cal_srv  -- performs extrinsic calibration when either the target or the camera is mounted on the end of arm tooling
stereo_cal_srv -- performs stereo extrinsic cal using a robot (target or pair mounted on end of arm tooling)

extrinsic_cal helper nodes:
camera_observer_scene_trigger    -- provides a trigger for a scene when the camera can locate a target
manual_calt_adjust               -- Allows using the keyboard to adjust a mutable transform. typing x or X increases or decreases translation in x etc
mutable_joint_state_publisher    -- Publishes the joint states for all joints defined in the input yaml file, allows update and save.
ros_robot_trigger_action_service -- Service which triggers a scene by moving a robot (using Moveit!) to a new pose
trigger_serice                   -- A very simple example of a scene trigger. Set a parameter to trigger a scene
target_display_node              -- publishes a marker array that looks like a modified circle grid target (no stl required)

intrinsic_cal calibration nodes:
ical_srv        -- intrinsic calibration using basic services
rail_cal        -- dedicated rail calibration
robocyl_ical    -- intrinsic calibraton on the robocylinder
robocyl_vcal    -- verifies the intrinsic calibration results using robocylinder
robocyl_scal    -- uses robocylinder to perform stereo calibration
robocyl_ical_8d -- an improved version of ical_srv and robocyl_ical included determination of axis of motion for rail. Target on rail only!!
robot_ical      -- intrinsic calibration on a robot. This is very useful if you don't want to take the camera off the robot, but not as accurate.

caljob_creator nodes:
caljob_creator  -- writes a yaml file using the current joint values to create a scene ( only use if you are trying to use extrinsic_cal/service_node
joint_traj_node -- A node that maintains a list of robot poses that works in conjuction with the basic calibration service interface to orchestrate calibrations

rgbd_depth_correction nodes:
depth_calibration      -- calibrates a 3D camera using a rail (usually a manual rail because it has to be fairly long)
depth_correctonNodelet -- a nodelet that subscribes to a cloud and corrects the depth using the calibration found by the depth_calibration node

stand_alone_gui nodes:
ical_gui -- as stand alone gui that makes using the basic calibration service easy

target_finder nodes:
target_locator_srv -- finds the target in a camera image and computes the target to camera transform
stereo_locator_srv -- finds the target in a stereo pair and computes the target to left camera transform
target_gen         -- generates a target yaml file (NO LONGER NEEDED)
call_service       -- continuously calls the target locator service and publishes the transform results
dual_call_service  -- continuously calls the target locator service for two different cameras and publishes the transform results for each
stereo_pose_stats  -- calls the stereo_locator_srv 30 times and computes statistics on the results

calibration_guis: what is it?
This package creates two plugin panels for rviz. The first is an interface to the joint_traj node in caljob creator. The second is an interface to the standard calibration call service.


The best place to start in using this set of packages is with one of the basic call service calibration routines. Each of these routines advertises the following services:
1. Start  -- resets all data
2. Obs    -- collects an observation
3. Run    -- runs the optimization
4. Save   -- save the results, For intrinsic calibration it pushes to camera_info_manager. For extrinsic, it pushes to the mutable joint state publisher. 
5. Import -- imports the data saved from a previous calibration.
6. Cov    -- computes and prints the covariance matrix

The following are basic service call calibration nodes:
wrist_cal_srv  -- performs extrinsic calibration when either the target or the camera is mounted on the end of arm tooling
stereo_cal_srv -- performs stereo extrinsic cal using a robot (target or pair mounted on end of arm tooling)
ical_srv        -- intrinsic calibration using basic services (depricated)
robocyl_ical_8d -- an improved version of ical_srv and robocyl_ical included determination of axis of motion for rail. Target on rail only!!
robot_ical      -- intrinsic calibration on a robot. This is very useful if you don't want to take the camera off the robot, but not as accurate.

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

## Requires

### Ceres Optimizer

- With apt: `sudo apt install libceres-dev`
- With [rosdep](http://docs.ros.org/independent/api/rosdep/html/):
  `rosdep install industrial_extrinsic_cal` or
  `rosdep install --from-paths industrial_calibration/`

### Openni2
```
sudo apt-get install ros-kinetic-openni2-camera
sudo apt-get install ros-kinetic-openni2-launch
```

### Moveit
`sudo apt-get install ros-kinetic-moveit`

# Examples

## Single Basler on a rail
```
roslaunch robocyl_ical.launch
roslaunch robo_cylinder.launch
rosservice call /RobocylCalService "allowable_cost_per_observation: 0.25"
```

# Build
```
mkdir -p cal_ws/src
cd cal_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/industrial_calibration.git
cd ..
catkin build
```
