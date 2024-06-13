#pragma once

#include <industrial_calibration/target_finders/opencv/target_finder.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>
#include <QMainWindow>
#include <QDialog>

class QTreeWidgetItem;

namespace Ui
{
class CameraIntrinsicCalibration;
}

namespace industrial_calibration
{
class CameraIntrinsicResult;
class TargetFinderWidget;
class CameraIntrinsicsWidget;

class CameraIntrinsicCalibrationWidget : public QMainWindow
{
public:
  explicit CameraIntrinsicCalibrationWidget(QWidget* parent = nullptr);
  ~CameraIntrinsicCalibrationWidget();

  /**
   * @brief Loads the calibration configuration from file
   * @throws Exception on failure
   */
  void loadConfig(const std::string& config_file);

  /**
   * @brief Loads the calibration observations from file
   * @throws Exception on failure
   */
  void loadObservations(const std::string& obserations_file);

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

private:
  void onLoadConfig();
  void onLoadObservations();
  void onCalibrate();
  void onSaveResults();
  void onSaveROSFormat();

  void loadTargetFinder();
  void drawImage(QTreeWidgetItem* item, int col);

  Ui::CameraIntrinsicCalibration* ui_;
  TargetFinderWidget* target_finder_widget_;
  CameraIntrinsicsWidget* camera_intrinsics_widget_;

  boost_plugin_loader::PluginLoader loader_;
  TargetFinderFactoryOpenCV::ConstPtr factory_;

  std::shared_ptr<CameraIntrinsicResult> result_;
  TargetFinderOpenCV::ConstPtr target_finder_;
};

}  // namespace industrial_calibration
