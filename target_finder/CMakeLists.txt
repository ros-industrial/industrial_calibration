cmake_minimum_required(VERSION 2.8.3)
project(target_finder)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  industrial_extrinsic_cal
  message_generation
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  tf
)


#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fPIC")

find_package(Boost REQUIRED)

find_package(Ceres REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")


add_service_files(
  FILES
  target_locator.srv
  stereo_locator.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)


catkin_package(
   CATKIN_DEPENDS
     geometry_msgs
     industrial_extrinsic_cal
     message_runtime
     rosconsole
     roscpp
     roslib
     sensor_msgs
     std_msgs
     std_srvs
     tf
  DEPENDS
    Boost
    CERES
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


add_executable(target_gen src/nodes/grid_generator.cpp)
add_executable(call_service src/nodes/call_service.cpp)
add_executable(dual_call_service src/nodes/dual_camera_cs.cpp)
add_executable(target_locator_srv src/nodes/target_locator.cpp)
add_executable(stereo_locator_srv src/nodes/stereo_locator.cpp)
add_executable(stereo_pose_stats src/nodes/stereo_pose_stats.cpp)

add_dependencies(call_service ${catkin_EXPORTED_TARGETS} ${target_finder_EXPORTED_TARGETS})
add_dependencies(dual_call_service ${catkin_EXPORTED_TARGETS} ${target_finder_EXPORTED_TARGETS})
add_dependencies(target_locator_srv ${catkin_EXPORTED_TARGETS} ${target_finder_EXPORTED_TARGETS})
add_dependencies(stereo_locator_srv ${catkin_EXPORTED_TARGETS} ${target_finder_EXPORTED_TARGETS})
add_dependencies(stereo_pose_stats ${catkin_EXPORTED_TARGETS} ${target_finder_EXPORTED_TARGETS})

target_link_libraries(target_gen ${catkin_LIBRARIES})
target_link_libraries(call_service ${catkin_LIBRARIES})
target_link_libraries(dual_call_service ${catkin_LIBRARIES})
target_link_libraries(target_locator_srv ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(stereo_locator_srv ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(stereo_pose_stats ${catkin_LIBRARIES} ${CERES_LIBRARIES})


install(
  TARGETS
    dual_call_service
    call_service
    target_gen
    target_locator_srv
    stereo_locator_srv
    stereo_pose_stats
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
