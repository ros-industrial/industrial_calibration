add_library(
  ${PROJECT_NAME}_analysis SHARED
  src/camera_intrinsic_calibration_analysis.cpp
  src/extrinsic_hand_eye_calibration_analysis.cpp
  src/homography_analysis.cpp
  src/noise_qualification.cpp
  src/projection.cpp
  src/statistics.cpp)
target_include_directories(${PROJECT_NAME}_analysis PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_analysis PUBLIC ${PROJECT_NAME}_optimizations)
target_cxx_version(${PROJECT_NAME}_analysis PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_analysis
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_analysis
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include)

configure_component(
  COMPONENT
  analysis
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_analysis
  DEPENDENCIES
  "industrial_calibration COMPONENTS optimizations")
