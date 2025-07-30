#pragma once

#include <industrial_calibration/target_finders/opencv/target_finder.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>
#include <QMainWindow>
#include <QDialog>

class QTreeWidgetItem;

namespace Ui
{
class ExtrinsicHandEyeCalibration;
}

namespace industrial_calibration
{
class ExtrinsicHandEyeResult;
class TargetFinderWidget;
class CameraIntrinsicsWidget;
class TransformGuess;

/**
 * @brief Widget for performing extrinsic hand-eye calibration from a data set of 2D image observations.
 * @sa @ref page_extrinsic_hand_eye_calibration
 */
class ExtrinsicHandEyeCalibrationWidget : public QMainWindow
{
public:
  explicit ExtrinsicHandEyeCalibrationWidget(QWidget* parent = nullptr);
  ~ExtrinsicHandEyeCalibrationWidget();

  /**
   * @brief Loads the calibration configuration from file
   * @throws Exception on failure
   */
  void loadConfig(const std::string& config_file);

  /**
   * @brief Loads the calibration observations from file
   * @throws Exception on failure
   */
  void loadObservations(const std::string& observations_file);

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

protected:
  void closeEvent(QCloseEvent* event) override;

  void onLoadConfig();
  void onLoadObservations();
  void onCalibrate();
  void onSaveResults();

  void loadTargetFinder();
  void drawImage(QTreeWidgetItem* item, int col);

  Ui::ExtrinsicHandEyeCalibration* ui_;
  TargetFinderWidget* target_finder_widget_;
  CameraIntrinsicsWidget* camera_intrinsics_widget_;
  TransformGuess* camera_transform_guess_widget_;
  TransformGuess* target_transform_guess_widget_;

  boost_plugin_loader::PluginLoader loader_;
  TargetFinderFactoryOpenCV::ConstPtr factory_;

  std::shared_ptr<ExtrinsicHandEyeResult> result_;
  TargetFinderOpenCV::ConstPtr target_finder_;
};

}  // namespace industrial_calibration
