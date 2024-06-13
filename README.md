# Industrial Calibration

![Industrial Calibration](docs/extrinsic_hand_eye_calibration.png)

## Description
A set of tools for performing calibration between cameras and robots and analyzing the accuracy of the calibration.

See the [calibration primer](docs/primer.md) for more information on the tools in this repository

## Modules
- Core
    - Contains definitions for common data types (e.g., features, correspondences, observations, camera intrinsics, etc.) and interface classes.
- Optimizations
    - Contains type definitions, cost functions, and optimization algorithms for various calibration optimizations
- Analysis
    - Contains functions for analyzing the accuracy of sensors and various types of calibrations
- Target Finders
    - Contains implementations of the interface for finding calibration targets from sensor measurements (e.g., 2D images)
- GUI
    - Contains Qt-based GUI widgets for performing various types of calibrations
- Examples
    - Contains examples with real data for various types of calibrations, including analysis of the results

## GUI Applications
- [Extrinsic hand-eye calibration](docs/extrinsic_hand_eye_calibration.md)
- [Camera intrinsic calibration](docs/camera_intrinsic_calibration.md)

## Build

```commandLine
cd <workspace>
vcs import src < src/industrial_calibration/dependencies.repos
rosdep install --from-paths src -iry
colcon build
```
