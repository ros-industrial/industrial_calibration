# OpenCV
option(INDUSTRIAL_CALIBRATION_BUILD_OPENCV "Build OpenCV target finders" ON)
if(INDUSTRIAL_CALIBRATION_BUILD_OPENCV)
  message("Building OpenCV target finders")
  list(APPEND SUPPORTED_COMPONENTS target_finders_opencv)
  set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)
  add_subdirectory(opencv)
endif()
