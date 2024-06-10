# Industrial calibration gui
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(boost_plugin_loader REQUIRED)

# Add widget plugin
add_library(
  ${PROJECT_NAME}_gui SHARED
  # Utilities
  resources/resources.qrc
  src/configurable_widget.cpp
  src/aspect_ratio_pixmap_label.cpp
  include/industrial_calibration/gui/aspect_ratio_pixmap_label.h
  # Calibration widgets
  src/camera_intrinsics.ui
  src/camera_intrinsics.cpp
  src/transform_guess.ui
  src/transform_guess.cpp
  # Target finders
  src/charuco_grid_target_finder.ui
  src/charuco_grid_target_finder.cpp
  src/aruco_grid_target_finder.ui
  src/aruco_grid_target_finder.cpp
  src/modified_circle_grid_target_finder.ui
  src/modified_circle_grid_target_finder.cpp
  src/target_finder.ui
  src/target_finder.cpp
  # Extrinsic hand-eye calibration widget
  src/extrinsic_hand_eye_calibration_widget.ui
  src/extrinsic_hand_eye_calibration_widget.cpp
  # Camera intrinsic calibration widget
  src/camera_intrinsic_calibration_widget.ui
  src/camera_intrinsic_calibration_widget.cpp)
target_include_directories(${PROJECT_NAME}_gui PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")
target_link_libraries(
  ${PROJECT_NAME}_gui
  PUBLIC Qt5::Widgets
         boost_plugin_loader::boost_plugin_loader
         ${PROJECT_NAME}_analysis
         ${PROJECT_NAME}_target_finders_opencv)

# Add extrinsic hand-eye calibration application
add_executable(${PROJECT_NAME}_extrinsic_hand_eye_calibration_app src/app/extrinsic_hand_eye_calibration_app.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye_calibration_app PRIVATE ${PROJECT_NAME}_gui)

# Add camera intrinsic calibration application
add_executable(${PROJECT_NAME}_camera_intrinsic_calibration_app src/app/camera_intrinsic_calibration_app.cpp)
target_link_libraries(${PROJECT_NAME}_camera_intrinsic_calibration_app PRIVATE ${PROJECT_NAME}_gui)

# Install headers
install(DIRECTORY include/ DESTINATION include)

list(APPEND SUPPORTED_COMPONENTS gui)
set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)

configure_component(
  COMPONENT
  gui
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_gui
  ${PROJECT_NAME}_extrinsic_hand_eye_calibration_app
  ${PROJECT_NAME}_camera_intrinsic_calibration_app
  DEPENDENCIES
  "industrial_calibration COMPONENTS analysis target_finders_opencv"
  "Qt5 REQUIRED COMPONENTS Widgets"
  boost_plugin_loader)
