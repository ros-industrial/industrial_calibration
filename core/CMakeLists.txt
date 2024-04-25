find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME}_core SHARED src/camera_intrinsics.cpp src/dh_chain.cpp src/types.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen yaml-cpp)
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION 17)
target_clang_tidy(
  ${PROJECT_NAME}_core
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_CLANG_TIDY}
         WARNINGS_AS_ERRORS
         ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING}
         CHECKS
         ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_core
  INTERFACE
  ALL
  ENABLE ${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include)

list(APPEND SUPPORTED_COMPONENTS core)
set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)

configure_component(
  COMPONENT
  core
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_core
  DEPENDENCIES
  Eigen3
  yaml-cpp)
