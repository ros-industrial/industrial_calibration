cmake_minimum_required(VERSION 2.8.3)
project(calibration_guis)

## WARNING
# For the rviz plugin to work at runtime it may be necessary to configure the following 
# environment variables as this application depends on ros_scxml which uses QT5.12
# Therefore locate the installation path of QT5.12.* and setup your variables as follows
#export PATH="[path to QT5.12]/cmake:$PATH"
#export LD_LIBRARY_PATH=[path to QT5.12]/gcc_64/lib:$LD_LIBRARY_PATH

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  PrintSupport
)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  rviz
  roscpp
  rospy
  std_msgs
  industrial_extrinsic_cal
)

catkin_package(
  INCLUDE_DIRS
    include   
  LIBRARIES
    cal_gui
    traj
  CATKIN_DEPENDS
    pluginlib
    rviz
    roscpp
    std_msgs
  DEPENDS
    Qt5
    Qt5Core
    Qt5Gui
    Qt5Widgets
 )

set( cal_gui_SOURCES
  src/calibration_gui.cpp
  src/traj_panel.cpp
)

set( cal_gui_PANEL_SOURCES
)

set( cal_gui_widget_SOURCES
)

set( cal_gui_HEADERS
  include/calibration_guis/calibration_gui.h
  include/calibration_guis/traj_panel.h
)

set( cal_gui_panel_HEADERS
)

set( cal_gui_widget_HEADERS
)

set( cal_gui_UIS
)


# Run the MOC on the Qt-related UIs and headers
qt5_wrap_ui(cal_gui_UIS_H
  ${cal_gui_UIS}
)

qt5_wrap_cpp(cal_gui_MOCS
  ${cal_gui_HEADERS}
)

qt5_wrap_cpp(cal_gui_widget_MOCS
  ${cal_gui_widget_HEADERS}
)

qt5_wrap_cpp(cal_gui_panel_MOCS
  ${cal_gui_panel_HEADERS}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)


add_library(cal_gui
  ${cal_gui_SOURCES}
  ${cal_gui_MOCS} 
)
target_link_libraries(cal_gui
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5PrintSupport_LIBRARIES}
  ${catkin_LIBRARIES}
)


install(
  TARGETS
    cal_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
  
install(
  FILES
    cal_gui_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui)

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

