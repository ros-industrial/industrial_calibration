cmake_minimum_required(VERSION 2.8.3)
project(caljob_creator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  industrial_extrinsic_cal
)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)
if(NOT ${yaml_cpp_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

find_path(
  yaml_cpp_INCLUDE_DIR
  # bit of a trick
  NAMES yaml-cpp/yaml.h
  PATHS ${yaml_cpp_INCLUDE_DIRS}
)

find_library(
  yaml_cpp_LIBRARY
  NAMES ${yaml_cpp_LIBRARIES}
  PATHS ${yaml_cpp_LIBRARY_DIRS}
)


catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    tf
    industrial_extrinsic_cal
  DEPENDS
    OpenCV
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)


add_executable(caljob_creator_node src/main.cpp)
add_executable(joint_traj_node src/joint_traj.cpp)
add_dependencies(caljob_creator_node ${catkin_EXPORTED_TARGETS})
add_dependencies(joint_traj_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(caljob_creator_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${yaml_cpp_LIBRARY})
target_link_libraries(joint_traj_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  ${yaml_cpp_LIBRARY})


install(
  TARGETS
      caljob_creator_node
      joint_traj_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
