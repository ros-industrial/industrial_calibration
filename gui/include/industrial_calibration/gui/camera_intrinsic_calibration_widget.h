#pragma once

#include <industrial_calibration/gui/camera_calibration_data_manager_widget.h>

class QToolBar;

namespace industrial_calibration
{
class CameraIntrinsicResult;

/**
 * @brief Widget for performing camera intrinsic calibration from a data set of 2D image observations.
 * @sa @ref page_camera_intrinsic_calibration
 */
class CameraIntrinsicCalibrationWidget : public CameraCalibrationDataManagerWidget
{
  Q_OBJECT

public:
  /**
   * @brief Returns an HTML-formatted string with instructions on running the calibration
   */
  static std::string getInstructions();

  explicit CameraIntrinsicCalibrationWidget(QWidget* parent = nullptr);

  /**
   * @brief Loads the calibration configuration from file (defined in @ref s_camera_intrinsic_conf_def)
   * @throws Exception on failure
   */
  void loadConfig(const std::string& config_file);

  /**
   * @brief Performs the calibration
   * @throws Exception on failure
   */
  void calibrate();

  /**
   * @brief Saves the calibration results
   * @throws Exception on failure
   */
  void saveResults(const std::string& file) const;

  /**
   * @brief Saves the calibration results to a YAML file in a format compatible with ROS
   * @details Format definition https://wiki.ros.org/camera_calibration_parsers#File_formats
   * @throws Exception on failure
   */
  void saveROSFormat(const std::string& file) const;

  QAction* action_instructions;
  QAction* action_load_configuration;
  QAction* action_use_extrinsic_guesses;
  QAction* action_use_opencv;
  QAction* action_calibrate;
  QAction* action_save;
  QAction* action_save_ros_format;

  /**
   * @brief Configured tool bar with actions for the calibration
   * @details This tool bar is not added to the layout of the widget by default
   */
  QToolBar* tool_bar;

signals:
  /** @brief Signal emitted when calibration has successfully completed */
  void calibrationComplete(const CameraIntrinsicResult&);

protected:
  void onLoadConfig();
  void onCalibrate();
  void onSaveResults();
  void onSaveROSFormat();

  std::shared_ptr<CameraIntrinsicResult> result_;
};

}  // namespace industrial_calibration
