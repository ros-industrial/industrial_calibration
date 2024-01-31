# Industrial calibration gui
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Add widget plugin
add_library(
  ${PROJECT_NAME}_widget SHARED
  src/ic_widget.ui
  src/ic_widget.cpp
  include/widget/ic_widget.h
  src/camera_intrinsics.ui
  src/camera_intrinsics.cpp
  include/widget/camera_intrinsics.h
  src/charuco_target.ui
  src/charuco_target.cpp
  include/widget/charuco_target.h
  src/transform_guess.ui
  src/transform_guess.cpp
  include/widget/transform_guess.h)
target_include_directories(${PROJECT_NAME}_widget PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_widget PUBLIC
  Qt5::Widgets 
  boost_plugin_loader::boost_plugin_loader
  ${PROJECT_NAME})

# add executable for widget
add_executable(${PROJECT_NAME}_app src/ic_app.cpp)
# target_include_directories(${PROJECT_NAME}_app PRIVATE
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_app PRIVATE ${PROJECT_NAME}_widget)

# Install executables
# install(TARGETS ${PROJECT_NAME}_app EXPORT ${PROJECT_NAME}-targets DESTINATION bin)
configure_component(
  COMPONENT gui 
  NAMESPACE industrial_calibration 
  TARGETS ${PROJECT_NAME}_widget ${PROJECT_NAME}_app 
  DEPENDENCIES Qt5 boost_plugin_loader)