cmake_minimum_required(VERSION 2.8.3)
project(industrial_extrinsic_cal)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  moveit_ros_planning_interface
  rosconsole
  roscpp
  roslib
  roslint
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
)

find_package(Boost REQUIRED)

find_package(Ceres REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)

message("-- Found OpenCV version ${OpenCV_VERSION}: ${OpenCV_INCLUDE_DIRS}")

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

add_service_files(
  FILES
    calibrate.srv
    camera_observer_trigger.srv
    covariance.srv
    get_mutable_joint_states.srv
    set_mutable_joint_states.srv
    store_mutable_joint_states.srv
    user_accept.srv
    cal_srv_solve.srv
    FileOp.srv
)

add_action_files(
  DIRECTORY
   action
  FILES
   calibration.action
   manual_target_location.action
   manual_trigger.action
   robot_joint_values_trigger.action
   robot_pose_trigger.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/circle_grid_finder.cfg
)

catkin_package(
  LIBRARIES
    industrial_extrinsic_cal
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    cv_bridge
    geometry_msgs
    image_transport
    message_runtime
    moveit_ros_planning_interface
    rosconsole
    roscpp
    roslib
    roslint
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf_conversions
    pcl_ros
    pcl_conversions
  DEPENDS
    Boost
    CERES
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)
# target: main library
add_library(
  industrial_extrinsic_cal
  src/basic_types.cpp
  src/calibration_job_definition.cpp
  src/caljob_yaml_parser.cpp
  src/camera_definition.cpp
  src/camera_yaml_parser.cpp
  src/ceres_blocks.cpp
  src/ceres_costs_utils.cpp
  src/circle_detector.cpp
  src/observation_data_point.cpp
  src/observation_scene.cpp
  src/points_yaml_parser.cpp
  src/pose_yaml_parser.cpp
  src/ros_camera_observer.cpp
  src/transform_interface.cpp
  src/ros_transform_interface.cpp
  src/target.cpp
  src/targets_yaml_parser.cpp
  src/ros_target_display.cpp
  src/conical_pose_generator.cpp
)
add_dependencies(industrial_extrinsic_cal ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(
  industrial_extrinsic_cal
  ${yaml_cpp_LIBRARY}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)
# targets: other nodes
add_executable(camera_observer_scene_trigger    src/nodes/camera_observer_scene_trigger.cpp)
add_executable(manual_calt_adjust               src/nodes/manual_calt_adjuster.cpp)
add_executable(mutable_joint_state_publisher    src/nodes/mutable_joint_state_publisher.cpp)
add_executable(ros_robot_trigger_action_service src/nodes/ros_robot_scene_trigger_action_server.cpp)
add_executable(service_node                     src/nodes/calibration_service.cpp)
add_executable(trigger_service                  src/nodes/ros_scene_trigger_server.cpp)
add_executable(range_excal                      src/nodes/range_camera_excal.cpp)
add_executable(rangeNmono                       src/nodes/rangeNmono_excal.cpp)
add_executable(target_display                   src/nodes/target_display_node.cpp)
add_executable(wrist_cal_srv                    src/nodes/wrist_cal_srv.cpp)
add_executable(stereo_cal_srv                   src/nodes/stereo_cal_srv.cpp)
#add_executable(check_visible                    src/check_if_points_in_pic.cpp)

add_dependencies(camera_observer_scene_trigger    ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(mutable_joint_state_publisher    ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(ros_robot_trigger_action_service ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(service_node                     ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(trigger_service                  ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(range_excal                      ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(rangeNmono                       ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(target_display                   ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(wrist_cal_srv                    ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
add_dependencies(stereo_cal_srv                   ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})
#add_dependencies(check_visible                    ${catkin_EXPORTED_TARGETS} ${industrial_extrinsic_cal_EXPORTED_TARGETS})

target_link_libraries(camera_observer_scene_trigger industrial_extrinsic_cal ${catkin_LIBRARIES} ${yaml_cpp_LIBRARY} ${CERES_LIBRARIES})
target_link_libraries(manual_calt_adjust industrial_extrinsic_cal ${catkin_LIBRARIES})
target_link_libraries(mutable_joint_state_publisher ${catkin_LIBRARIES} ${yaml_cpp_LIBRARY})
target_link_libraries(ros_robot_trigger_action_service ${catkin_LIBRARIES})
target_link_libraries(service_node industrial_extrinsic_cal ${CERES_LIBRARIES})
target_link_libraries(target_display industrial_extrinsic_cal ${CERES_LIBRARIES})
target_link_libraries(trigger_service ${catkin_LIBRARIES})
target_link_libraries(range_excal industrial_extrinsic_cal ${CERES_LIBRARIES})
target_link_libraries(rangeNmono industrial_extrinsic_cal ${CERES_LIBRARIES})
target_link_libraries(wrist_cal_srv industrial_extrinsic_cal ${CERES_LIBRARIES})
target_link_libraries(stereo_cal_srv industrial_extrinsic_cal ${CERES_LIBRARIES})
#target_link_libraries(check_visible industrial_extrinsic_cal ${CERES_LIBRARIES})

install(
  TARGETS
    camera_observer_scene_trigger
    industrial_extrinsic_cal
    manual_calt_adjust
    mutable_joint_state_publisher
    ros_robot_trigger_action_service
    service_node
    trigger_service
    range_excal
    rangeNmono
    target_display
    wrist_cal_srv
    stereo_cal_srv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    config launch targets urdf yaml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(ROSLINT_CPP_OPTS
  "--filter=-whitespace,-build/header_guard,-build/include_order,-readability/streams,-runtime/references")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.h)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
