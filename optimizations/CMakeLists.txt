find_package(Ceres REQUIRED)

add_library(
  ${PROJECT_NAME}_optimizations SHARED
  # Optimization Utilities
  src/ceres_math_utilities.cpp
  src/covariance_types.cpp
  src/covariance_analysis.cpp
  # Optimizations
  src/pnp.cpp
  src/camera_intrinsic.cpp
  src/dh_chain_kinematic_calibration.cpp
  src/extrinsic_hand_eye.cpp
  src/extrinsic_multi_static_camera.cpp
  src/multi_camera_pnp.cpp)
target_include_directories(${PROJECT_NAME}_optimizations PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_optimizations SYSTEM PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_optimizations PUBLIC ${PROJECT_NAME}_core ${CERES_LIBRARIES})
target_cxx_version(${PROJECT_NAME}_optimizations PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_optimizations
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_optimizations
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include)

list(APPEND SUPPORTED_COMPONENTS optimizations)
set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)

configure_component(
  COMPONENT
  optimizations
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_optimizations
  DEPENDENCIES
  Ceres
  "industrial_calibration COMPONENTS core")
