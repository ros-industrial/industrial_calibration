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

class ExtrinsicHandEyeCalibrationWidget : public QMainWindow
{
public:
  explicit ExtrinsicHandEyeCalibrationWidget(QWidget* parent = nullptr);
  ~ExtrinsicHandEyeCalibrationWidget();

private:
  void loadConfig();
  void loadObservations();
  void calibrate();

  void loadTargetFinder();
  void drawImage(QTreeWidgetItem* item, int col);
  void saveResults();

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
