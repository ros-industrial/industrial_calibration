find_package(boost_plugin_loader REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
  ${PROJECT_NAME}_target_finders_opencv SHARED
  src/utils.cpp
  src/circle_detector.cpp
  src/target_finder.cpp
  src/modified_circle_grid_target_finder.cpp
  src/aruco_grid_target_finder.cpp
  src/charuco_grid_target_finder.cpp)
target_include_directories(
  ${PROJECT_NAME}_target_finders_opencv PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                               "$<INSTALL_INTERFACE:include>")
target_link_libraries(
  ${PROJECT_NAME}_target_finders_opencv
  PUBLIC ${PROJECT_NAME}_core
         opencv_core
         opencv_aruco
         opencv_features2d
         opencv_imgproc
         opencv_imgcodecs)
target_compile_definitions(
  ${PROJECT_NAME}_target_finders_opencv
  PUBLIC TARGET_FINDER_SECTION="target" INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV="INDUSTRIAL_CALIBRATION_plugins"
         INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES="${PROJECT_NAME}_plugins_opencv")
target_cxx_version(${PROJECT_NAME}_target_finders_opencv PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_target_finders_opencv
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_target_finders_opencv
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Plugin loader
add_library(${PROJECT_NAME}_plugins_opencv SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins_opencv PUBLIC ${PROJECT_NAME}_target_finders_opencv
                                                            boost_plugin_loader::boost_plugin_loader)
target_cxx_version(${PROJECT_NAME}_plugins_opencv PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_plugins_opencv
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_plugins_opencv
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install the modified circle grid target generator script
install(FILES script/modified_circle_grid_target_generator.py DESTINATION bin)

configure_component(
  COMPONENT
  target_finders_opencv
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_target_finders_opencv
  ${PROJECT_NAME}_plugins_opencv
  DEPENDENCIES
  boost_plugin_loader
  OpenCV
  "industrial_calibration COMPONENTS core")
