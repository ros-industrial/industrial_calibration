find_package(boost_plugin_loader REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
  ${PROJECT_NAME}_target_finders SHARED
  src/circle_detector.cpp
  src/target_finder.cpp
  src/modified_circle_grid_target_finder.cpp
  src/aruco_grid_target_finder.cpp
  src/charuco_grid_target_finder.cpp)
target_include_directories(
  ${PROJECT_NAME}_target_finders PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_target_finders SYSTEM PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}_target_finders
  PUBLIC ${PROJECT_NAME}
         opencv_core
         opencv_aruco
         opencv_features2d
         opencv_imgproc
         opencv_imgcodecs)
target_compile_definitions(
  ${PROJECT_NAME}_target_finders
  PUBLIC TARGET_FINDER_SECTION="target" INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV="INDUSTRIAL_CALIBRATION_PLUGINS"
         INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES="${PROJECT_NAME}_plugins")
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_target_finders
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_target_finders
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Plugin loader
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME}_target_finders
                                                     boost_plugin_loader::boost_plugin_loader)
target_cxx_version(${PROJECT_NAME}_plugins PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_plugins
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_plugins
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include)

configure_component(
  COMPONENT
  target_finders
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_target_finders
  ${PROJECT_NAME}_plugins
  DEPENDENCIES
  boost_plugin_loader
  OpenCV
  "industrial_calibration COMPONENTS core")
