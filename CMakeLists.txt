cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED
  # Common
  src/dh_chain.cpp
  # Optimization Utilities
  src/optimizations/utils/ceres_math_utilities.cpp
  # Optimizations
  src/optimizations/pnp.cpp
  src/optimizations/camera_intrinsic.cpp
  src/optimizations/dh_chain_kinematic_calibration.cpp
  src/optimizations/extrinsic_hand_eye.cpp
  src/optimizations/extrinsic_multi_static_camera.cpp
  src/optimizations/multi_camera_pnp.cpp
  # Optimization Analysis
  src/optimizations/analysis/camera_intrinsic_calibration_analysis.cpp
  src/optimizations/analysis/covariance_types.cpp
  src/optimizations/analysis/covariance_analysis.cpp
  src/optimizations/analysis/homography_analysis.cpp
  src/optimizations/analysis/statistics.cpp
  src/optimizations/analysis/noise_qualification.cpp
  # Target Finders
  src/target_finders/utils/circle_detector.cpp
  src/target_finders/utils/utils.cpp
  src/target_finders/target_finder.cpp
  src/target_finders/modified_circle_grid_target_finder.cpp
  src/target_finders/aruco_grid_target_finder.cpp
  src/target_finders/charuco_grid_target_finder.cpp
  src/target_finders/target_finder_plugins.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${CERES_LIBRARIES}
    yaml-cpp
    opencv_core
    opencv_aruco
    opencv_features2d
    opencv_imgproc
    opencv_imgcodecs)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 17)
target_clang_tidy(${PROJECT_NAME}
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Unit tests
if(${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Calibration examples
add_subdirectory(examples)

# Package configuration
configure_package(
  NAMESPACE
    industrial_calibration
  TARGETS
    ${PROJECT_NAME}
  DEPENDENCIES
    Eigen3
    OpenCV
    Ceres
    yaml-cpp)
