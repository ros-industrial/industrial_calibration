cmake_minimum_required(VERSION 2.8.3)
project(rgbd_depth_correction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  message_generation
  sensor_msgs
  tf
  geometry_msgs
  tf2_bullet
  rospy
  target_finder
  tf_conversions
  nodelet
  cv_bridge
  industrial_extrinsic_cal
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)

# Ceres
FIND_PACKAGE(Ceres REQUIRED)
IF (CERES_FOUND)
  MESSAGE("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")
ENDIF (CERES_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files( FILES)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   pcl_msgs#   sensor_msgs#   visualization_msgs
# )
#generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)


# macro definitions required by simple_message library
add_definitions(-DROS=1)  #build using ROS libraries
#add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_calibration
  CATKIN_DEPENDS message_runtime pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs
                 tf geometry_msgs nodelet
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(rgbd_depth_correction
  src/depth_correction.cpp
)
target_link_libraries(rgbd_depth_correction ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)
add_dependencies(rgbd_depth_correction ${PROJECT_NAME}_generate_messages_cpp)


add_executable(depth_calibration src/depth_calibration.cpp)

target_link_libraries(depth_calibration ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)

add_dependencies(depth_calibration ${PROJECT_NAME}_generate_messages_cpp)




