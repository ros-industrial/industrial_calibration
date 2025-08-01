find_package(boost_plugin_loader REQUIRED)

# Camera intrinsic calibration example
add_executable(${PROJECT_NAME}_camera_intrinsic camera_intrinsic.cpp)
target_link_libraries(
  ${PROJECT_NAME}_camera_intrinsic
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_camera_intrinsic PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_camera_intrinsic PUBLIC VERSION 17)

# Hand-eye calibration example
add_executable(${PROJECT_NAME}_extrinsic_hand_eye extrinsic_hand_eye.cpp)
target_link_libraries(
  ${PROJECT_NAME}_extrinsic_hand_eye
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_extrinsic_hand_eye
                           PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_extrinsic_hand_eye PUBLIC VERSION 17)

# Kinematic calibration example
add_executable(${PROJECT_NAME}_kinematic_calibration kinematic_calibration.cpp)
target_link_libraries(${PROJECT_NAME}_kinematic_calibration PRIVATE ${PROJECT_NAME}_analysis
                                                                    ${PROJECT_NAME}_target_finders_opencv)
target_compile_definitions(${PROJECT_NAME}_kinematic_calibration
                           PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_kinematic_calibration PUBLIC VERSION 17)

# Noise qualification example
add_executable(${PROJECT_NAME}_noise_qualification_2d noise_qualification_2d.cpp)
target_link_libraries(
  ${PROJECT_NAME}_noise_qualification_2d
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_noise_qualification_2d
                           PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_noise_qualification_2d PUBLIC VERSION 17)

# PnP example
add_executable(${PROJECT_NAME}_pnp pnp.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pnp
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_pnp PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_pnp PUBLIC VERSION 17)

# Build unit tests from the examples
if(${INDUSTRIAL_CALIBRATION_ENABLE_TESTING})
  find_gtest()

  # Camera intrinsic calibration
  add_gtest_discover_tests(${PROJECT_NAME}_camera_intrinsic)
  add_dependencies(run_tests ${PROJECT_NAME}_camera_intrinsic)
  target_link_libraries(${PROJECT_NAME}_camera_intrinsic PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_camera_intrinsic PUBLIC INDUSTRIAL_CALIBRATION_ENABLE_TESTING)

  # Hand-eye calibration
  add_gtest_discover_tests(${PROJECT_NAME}_extrinsic_hand_eye)
  add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_hand_eye)
  target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_extrinsic_hand_eye PUBLIC INDUSTRIAL_CALIBRATION_ENABLE_TESTING)

  # Kinematic calibration
  add_gtest_discover_tests(${PROJECT_NAME}_kinematic_calibration)
  add_dependencies(run_tests ${PROJECT_NAME}_kinematic_calibration)
  target_link_libraries(${PROJECT_NAME}_kinematic_calibration PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_kinematic_calibration PUBLIC INDUSTRIAL_CALIBRATION_ENABLE_TESTING)

  # Noise qualification
  add_gtest_discover_tests(${PROJECT_NAME}_noise_qualification_2d)
  add_dependencies(run_tests ${PROJECT_NAME}_noise_qualification_2d)
  target_link_libraries(${PROJECT_NAME}_noise_qualification_2d PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_noise_qualification_2d PUBLIC INDUSTRIAL_CALIBRATION_ENABLE_TESTING)

  # PnP
  add_gtest_discover_tests(${PROJECT_NAME}_pnp)
  add_dependencies(run_tests ${PROJECT_NAME}_pnp)
  target_link_libraries(${PROJECT_NAME}_pnp PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_pnp PUBLIC INDUSTRIAL_CALIBRATION_ENABLE_TESTING)
endif()

# Install the data directory
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

list(APPEND SUPPORTED_COMPONENTS examples)
set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)

configure_component(
  COMPONENT
  examples
  NAMESPACE
  industrial_calibration
  TARGETS
  ${PROJECT_NAME}_camera_intrinsic
  ${PROJECT_NAME}_extrinsic_hand_eye
  ${PROJECT_NAME}_kinematic_calibration
  ${PROJECT_NAME}_noise_qualification_2d
  ${PROJECT_NAME}_pnp
  DEPENDENCIES
  "industrial_calibration COMPONENTS analysis target_finders_opencv")

# Install the executables for access by ROS
install(
  TARGETS ${PROJECT_NAME}_camera_intrinsic
          ${PROJECT_NAME}_extrinsic_hand_eye
          ${PROJECT_NAME}_kinematic_calibration
          ${PROJECT_NAME}_noise_qualification_2d
          ${PROJECT_NAME}_pnp
  DESTINATION lib/${PROJECT_NAME})
