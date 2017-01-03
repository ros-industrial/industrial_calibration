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
)

find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    tf
  DEPENDS
    OpenCV
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(caljob_creator_node src/main.cpp)
add_dependencies(caljob_creator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(caljob_creator_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


install(
  TARGETS
    caljob_creator_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
