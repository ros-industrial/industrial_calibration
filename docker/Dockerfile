ARG TAG=jammy
FROM ubuntu:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install
RUN apt update -y -qq \
  && apt install -y -qq cmake curl git python3

# Add ROS2 sources to install ROS infrastructure tools
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install and configure ROS infrastructure tools
RUN apt update -y -qq \
  && apt install -y -qq python3-vcstool python3-colcon-common-extensions python3-rosdep \
  && rosdep init \
  && rosdep update

# Build the repository
# tmpfs mount a directory for the workspace so as not to unnecessarily copy source code into the docker image for the repository dependencies
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/industrial_calibration
ARG INSTALL_DIR=/opt/industrial_calibration
RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/industrial_calibration \
  vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/industrial_calibration/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
  && cd ${WORKSPACE_DIR} \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install/ ${INSTALL_DIR}

# Set the entrypoint to source the workspace
COPY docker/extrinsic_calibration_gui.sh /extrinsic_calibration_gui.sh
COPY docker/intrinsic_calibration_gui.sh /intrinsic_calibration_gui.sh

ENTRYPOINT ["./extrinsic_calibration_gui.sh"]
