cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

set(SUPPORTED_COMPONENTS "")

add_subdirectory(core)
list(APPEND SUPPORTED_COMPONENTS core)

add_subdirectory(optimizations)
list(APPEND SUPPORTED_COMPONENTS optimizations)

add_subdirectory(analysis)
list(APPEND SUPPORTED_COMPONENTS analysis)

add_subdirectory(target_finders)
# The target_finders subdirectory manages the addition of its own supported components

# Unit tests
if(${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Examples
add_subdirectory(examples)
list(APPEND SUPPORTED_COMPONENTS examples)

# Package configuration
configure_package(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})
