cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
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

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Package configuration
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE "industrial_calibration::" DESTINATION lib/cmake/${PROJECT_NAME})
export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "industrial_calibration::" FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
configure_package()
