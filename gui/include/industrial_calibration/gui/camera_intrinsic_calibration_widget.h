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

  void loadConfig(const std::string& config_file);
  void loadObservations(const std::string& obserations_file);

private:
  void loadConfig();
  void loadObservations();
  void calibrate();

  void loadTargetFinder();
  void drawImage(QTreeWidgetItem* item, int col);
  void saveResults();

  Ui::CameraIntrinsicCalibration* ui_;
  TargetFinderWidget* target_finder_widget_;
  CameraIntrinsicsWidget* camera_intrinsics_widget_;

  boost_plugin_loader::PluginLoader loader_;
  TargetFinderFactoryOpenCV::ConstPtr factory_;

  std::shared_ptr<CameraIntrinsicResult> result_;
  TargetFinderOpenCV::ConstPtr target_finder_;
};

}  // namespace industrial_calibration
