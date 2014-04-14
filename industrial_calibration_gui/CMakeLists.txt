cmake_minimum_required(VERSION 2.8.3)
project(industrial_calibration_gui)

find_package(catkin REQUIRED COMPONENTS rosconsole)


catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/calibration_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
