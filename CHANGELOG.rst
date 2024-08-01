^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#200 <https://github.com/marip8/industrial_calibration/issues/200>`_ from marip8/update/install-execs-ros
  Installed executables in directory accessible by ROS
* Installed executables in directory accessible by ROS
* Merge pull request `#198 <https://github.com/marip8/industrial_calibration/issues/198>`_ from DavidMerzJr/link-to-ros-interfaces
  README: Add link to ROS interfaces
* README: Add link to ROS interfaces
* Merge pull request `#196 <https://github.com/marip8/industrial_calibration/issues/196>`_ from marip8/update/ci
  CI Update
* Run CI on push to main branch
* Contributors: David Merz, Jr, Michael Ripperger

1.0.0 (2024-06-17)
------------------
* Added exit dialog questions to calibration widgets
* Add configuration of OpenCV algorithm flag to camera intrinsic calibration
* Updated GUI applications to run semi-headless when all input file names are specified
* Add option to use OpenCV calibration
* Added camera intrinsic calibration widget and application
* Added options to extrinsic hand eye calibration app for loading configuration and data files from CLI arguments
* Added target finder widget
* Set lower bound on focal lengths and camera center parameters
* Added YAML serialization to camera intrinsic optimization
* Set covariance null space rank option; add try catch around calculation of covariance
* Added GUI package
* Ported PnP example from https://github.com/Jmeyer1292/robot_cal_tools/tree/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported noise qualification example from https://github.com/Jmeyer1292/robot_cal_tools/tree/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported kinematic calibration example from https://github.com/Jmeyer1292/robot_cal_tools/tree/2fd5b85de098fc2e97d06d51addd638304e0610e
* Added target on wrist hand-eye calibration example data set using ChArUco grid target
* Added example data for camera on wrist hand-eye calibration example using a modified circle grid target
* Ported extrinsic hand-eye calibration example for camera on wrist using modified circle grid target and target on wrist using ChArUco grid target from https://github.com/Jmeyer1292/robot_cal_tools/tree/2fd5b85de098fc2e97d06d51addd638304e0610e
* Added convenience functions to target finder for finding correspondences
* Revised target finder plugin
* Added ArUco grid target finder plugin implementation
* Ported classes for future target finder plugins from https://github.com/Jmeyer1292/robot_cal_tools/tree/2fd5b85de098fc2e97d06d51addd638304e0610e
* Updated and renamed image utilities file
* Updated code to use industrial calibration specific exceptions
* Ported target finder test from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported target finders from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported target finder definition from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported optimization unit tests from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported serialization from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Moved Ceres math utilities to utils directory
* Ported calibration analysis tools from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported optimizations from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported optimization utilities from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported calibration cost functions from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Created new file for industrial calibration specific exceptions
* Ported DH chain definition from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported math utilities from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Ported calibration type definitions from https://github.com/Jmeyer1292/robot_cal_tools/commit/2fd5b85de098fc2e97d06d51addd638304e0610e
* Initial commit for the industrial calibration core package
* Contributors: Michael Ripperger
