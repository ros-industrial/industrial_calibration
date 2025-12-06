cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

set(SUPPORTED_COMPONENTS "")

add_subdirectory(core)
add_subdirectory(optimizations)
add_subdirectory(analysis)
add_subdirectory(target_finders)

# GUI
option(INDUSTRIAL_CALIBRATION_BUILD_GUI "Build GUI" ON)
if(INDUSTRIAL_CALIBRATION_BUILD_GUI)
  message("Building GUI")
  add_subdirectory(gui)
endif()

# Unit tests
if(${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Examples
option(INDUSTRIAL_CALIBRATION_BUILD_EXAMPLES "Build examples" ON)
if(INDUSTRIAL_CALIBRATION_BUILD_EXAMPLES)
  message("Building examples")
  add_subdirectory(examples)
endif()

# Package configuration
configure_package(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})
