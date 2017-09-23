cmake_minimum_required(VERSION 2.8.3)
project(rgbd_depth_correction)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  industrial_extrinsic_cal
  message_filters
  nodelet
  pcl_conversions
  pcl_msgs
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  std_srvs
  target_finder
  tf
  tf_conversions
)


find_package(Boost REQUIRED COMPONENTS thread)

find_package(Ceres REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)
if(NOT ${yaml_cpp_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# resolve include path
find_path(
  yaml_cpp_INCLUDE_DIR
  # bit of a trick
  NAMES yaml-cpp/yaml.h
  PATHS ${yaml_cpp_INCLUDE_DIRS}
)

# determine absolute path to library
find_library(
  yaml_cpp_LIBRARY
  NAMES ${yaml_cpp_LIBRARIES}
  PATHS ${yaml_cpp_LIBRARY_DIRS}
)


catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    industrial_extrinsic_cal
    message_filters
    nodelet
    pcl_conversions
    pcl_msgs
    pcl_ros
    pluginlib
    roscpp
    sensor_msgs
    std_srvs
    target_finder
    tf
    tf_conversions
  DEPENDS
    Boost
    CERES
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)


add_library(rgbd_depth_correction src/depth_correction.cpp)
target_link_libraries(rgbd_depth_correction ${catkin_LIBRARIES} ${yaml_cpp_LIBRARY} ${CERES_LIBRARIES})
add_dependencies(rgbd_depth_correction ${catkin_EXPORTED_TARGETS})

add_executable(depth_calibration src/depth_calibration.cpp)
target_link_libraries(depth_calibration ${catkin_LIBRARIES} ${yaml_cpp_LIBRARY} ${CERES_LIBRARIES})
add_dependencies(depth_calibration ${catkin_EXPORTED_TARGETS})


install(
  TARGETS
    depth_calibration
    rgbd_depth_correction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    nodelets.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    config launch urdf yaml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
