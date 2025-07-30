#pragma once

#include <industrial_calibration/gui/camera_calibration_data_manager_widget.h>

class QToolBar;

namespace industrial_calibration
{
class ExtrinsicHandEyeResult;
class TransformGuess;

/**
 * @brief Widget for performing extrinsic hand-eye calibration from a data set of 2D image observations.
 * @sa @ref page_extrinsic_hand_eye_calibration
 */
class ExtrinsicHandEyeCalibrationWidget : public CameraCalibrationDataManagerWidget
{
  Q_OBJECT

public:
  /**
   * @brief Returns an HTML-formatted string with instructions on running the calibration
   */
  static std::string getInstructions();

  explicit ExtrinsicHandEyeCalibrationWidget(QWidget* parent = nullptr);

  /**
   * @brief Loads the calibration configuration from file (defined in @ref s_extrinsic_hand_eye_conf_def)
   * @throws Exception on failure
   */
  void loadConfig(const std::string& config_file);

  /**
   * @brief Performs the calibration
   * @throws Exception on failure
   */
  void calibrate();

  /**
   * @brief Saves results of the calibration
   * @throws Exception on failure
   */
  void saveResults(const std::string& file);

  TransformGuess* camera_transform_guess_widget_;
  TransformGuess* target_transform_guess_widget_;
  QAction* action_instructions;
  QAction* action_load_configuration;
  QAction* action_camera_mount_to_camera;
  QAction* action_target_mount_to_target;
  QAction* action_static_camera;
  QAction* action_save;
  QAction* action_calibrate;

  /**
   * @brief Configured tool bar with actions for the calibration
   * @details This tool bar is not added to the layout of the widget by default
   */
  QToolBar* tool_bar;

signals:
  /** @brief Signal emitted when calibration has successfully completed */
  void calibrationComplete(const ExtrinsicHandEyeResult&);

protected:
  void onLoadConfig();
  void onCalibrate();
  void onSaveResults();

  std::shared_ptr<ExtrinsicHandEyeResult> result_;
};

}  // namespace industrial_calibration
