# Industrial calibration gui
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(boost_plugin_loader REQUIRED)

# Add widget plugin
add_library(
  ${PROJECT_NAME}_widget SHARED
  # Utilities
  include/widget/configurable_widget.h
  src/widgets/configurable_widget.cpp
  src/widgets/camera_intrinsics.ui
  src/widgets/aspect_ratio_pixmap_label.cpp
  include/widget/aspect_ratio_pixmap_label.h
  # Calibration widgets
  src/widgets/camera_intrinsics.cpp
  include/widget/camera_intrinsics.h
  src/widgets/transform_guess.ui
  src/widgets/transform_guess.cpp
  include/widget/transform_guess.h
  # Target finders
  src/widgets/charuco_grid_target_finder.ui
  src/widgets/charuco_grid_target_finder.cpp
  include/widget/charuco_grid_target_finder.h
  src/widgets/aruco_grid_target_finder.ui
  src/widgets/aruco_grid_target_finder.cpp
  include/widget/aruco_grid_target_finder.h
  src/widgets/modified_circle_grid_target_finder.ui
  src/widgets/modified_circle_grid_target_finder.cpp
  include/widget/modified_circle_grid_target_finder.h
  # Extrinsic hand-eye calibration widget
  src/widgets/extrinsic_hand_eye_calibration_widget.ui
  src/widgets/extrinsic_hand_eye_calibration_widget.cpp
  include/widget/extrinsic_hand_eye_calibration_widget.h
)
target_include_directories(${PROJECT_NAME}_widget PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_widget PUBLIC
  Qt5::Widgets
  boost_plugin_loader::boost_plugin_loader
  ${PROJECT_NAME}_analysis
  ${PROJECT_NAME}_target_finders_opencv)

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
  DEPENDENCIES "Qt5 REQUIRED COMPONENTS Widgets" boost_plugin_loader)
