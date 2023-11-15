# Industrial Calibration

![Industrial Calibration](docs/extrinsic_hand_eye_calibration.png)

## Description
A set of tools for performing calibration between cameras and robots and analyzing the accuracy of the calibration

## Build

```commandLine
cd <workspace>
vcs import src < src/industrial_calibration/dependencies.repos
rosdep install --from-paths src -iry
colcon build
```
