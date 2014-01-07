cmake_minimum_required(VERSION 2.8.3)
project(industrial_extrinsic_cal)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge tf image_transport roslint)


# Ceres
FIND_PACKAGE(Ceres REQUIRED)
IF (CERES_FOUND)
  MESSAGE("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")
ENDIF (CERES_FOUND)

# Eigen
FIND_PACKAGE(Eigen REQUIRED)
IF (EIGEN_FOUND)
  MESSAGE("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN_INCLUDE_DIRS}")
ENDIF (EIGEN_FOUND)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES industrial_extrinsic_cal
   CATKIN_DEPENDS roscpp std_msgs rosconsole
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(industrial_extrinsic_cal
#   src/${PROJECT_NAME}/industrial_extrinsic_cal.cpp
# )
add_library(industrial_extrinsic_cal_ceres
   src/calibration_job_definition.cpp
)
add_library(industrial_extrinsic_cal
   src/ros_camera_observer.cpp
   src/camera_definition.cpp
   src/observation_scene.cpp
   src/observation_data_point.cpp
   src/ceres_blocks.cpp
#   src/calibration_job_class.cpp
)

## Declare a cpp executable
# add_executable(industrial_extrinsic_cal_node src/industrial_extrinsic_cal_node.cpp)
add_executable(mono_ex_cal src/mono_ex_cal.cpp)
add_executable(cal_job src/test_cal_job.cpp)
add_executable(test_obs src/test_ros_cam_obs.cpp)
add_executable(boeing_eval src/ba_ex_cal.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(industrial_extrinsic_cal_node industrial_extrinsic_cal_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(industrial_extrinsic_cal_ceres yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(mono_ex_cal ${CERES_LIBRARIES} )
target_link_libraries(boeing_eval ${CERES_LIBRARIES}  ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(test_obs industrial_extrinsic_cal ${catkin_LIBRARIES})
target_link_libraries(cal_job industrial_extrinsic_cal industrial_extrinsic_cal_ceres ${CERES_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS industrial_extrinsic_cal industrial_extrinsic_cal_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_industrial_extrinsic_cal.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## ROSlint checks
## For more information see: http://wiki.ros.org/roslint
##
## CPPlint options justifications
## - whitespace/* - errors are ignored, it is assumed that the follwing eclipse
##   format file has been applied: 
##   http://wiki.ros.org/IDEs?action=AttachFile&do=get&target=ROS_format.xml
## - build/header_guard - #ifdef/#ifndef naming doesn't match ROS sytle
## - readability/stream - does not apply to logging (the typical use in ROS)
## - runtime/references - pass by reference is allowed for output parameters
set(ROSLINT_CPP_OPTS 
"--filter=-whitespace,-build/header_guard,-build/include_order,-readability/streams,-runtime/references")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.h)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
