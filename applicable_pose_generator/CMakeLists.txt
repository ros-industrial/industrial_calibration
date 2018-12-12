cmake_minimum_required(VERSION 2.8.3)
project(applicable_pose_generator)

add_compile_options(-std=c++11 -Wall -Wextra)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tesseract_ros
  kdl_parser
)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
    ${PROJECT_NAME}_kdl
    ${orocos_kdl_LIBRARIES}
   CATKIN_DEPENDS
    tesseract_ros
    roscpp
    rospy
    std_msgs
    kdl_parser
   DEPENDS
    orocos_kdl
    EIGEN3
    Boost
    OCTOMAP
)

set(SRC_FILES
  srs/kdl_chain_kin.cpp
  srs/chainiksolverpos_lma.cpp
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_kdl
  src/pose_reachability_filter.cpp
)
target_link_libraries(${PROJECT_NAME}_kdl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})


 


