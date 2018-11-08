cmake_minimum_required(VERSION 2.8.3)
project(intrinsic_cal)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  industrial_extrinsic_cal
  message_generation
  rosconsole
  roscpp
  roslib
  std_msgs
  std_srvs
  robo_cylinder
  dynamic_reconfigure
)

find_package(Ceres REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fPIC")
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
  rail_ical_run.srv
  rail_scal_run.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/observe.cfg
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    industrial_extrinsic_cal
    message_runtime
    rosconsole
    roscpp
    roslib
    std_msgs
    std_srvs
  DEPENDS
    CERES
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)

add_executable(rail_ical src/rail_cal.cpp)
add_executable(robocyl_ical src/robocyl_ical.cpp)
add_executable(robocyl_vcal src/robocyl_vcal.cpp)
add_executable(robocyl_scal src/robocyl_scal.cpp)
add_executable(robocyl_ical_8D src/robocyl_ical_8D.cpp)
add_executable(robot_ical src/robot_ical.cpp)
add_executable(ical_srv src/ical_srv.cpp)
add_dependencies(rail_ical ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(robocyl_ical ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(robocyl_vcal ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(robocyl_scal ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(robocyl_ical_8D ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(robot_ical ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(ical_srv ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rail_ical ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(robocyl_ical ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(robocyl_vcal ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(robocyl_scal ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(robocyl_ical_8D ${catkin_LIBRARIES} ${CERES_LIBRARIES} )
target_link_libraries(robot_ical ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${yaml_cpp_LIBRARY})
target_link_libraries(ical_srv ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${yaml_cpp_LIBRARY})
 

install(
  TARGETS
    rail_ical
    robocyl_ical
    robocyl_vcal
    robocyl_scal
    robot_ical
    ical_srv
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch yaml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
    asus_ir_cal.rviz
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
