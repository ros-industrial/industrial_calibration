#pragma once

#include <industrial_calibration/core/types.h>
#include <industrial_calibration/core/camera_intrinsics.h>

#include <map>
#include <QString>
#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace Ui
{
class ExtrinsicHandEyeCalibrationConfiguration;
}

namespace industrial_calibration
{
class ConfigurableWidgetDialog;

class ExtrinsicHandEyeCalibrationConfigurationWidget : public QWidget
{
public:
  ExtrinsicHandEyeCalibrationConfigurationWidget(QWidget* parent = nullptr);
  virtual ~ExtrinsicHandEyeCalibrationConfigurationWidget();

  void load(const QString& file);
  YAML::Node getTargetFinderConfig() const;
  Eigen::Isometry3d getCameraMountToCameraGuess() const;
  Eigen::Isometry3d getTargetMountToTargetGuess() const;
  CameraIntrinsics getCameraIntrinsics() const;
  double getHomographyThreshold() const;
  bool getStaticCamera() const;

private:
  void save();

  Ui::ExtrinsicHandEyeCalibrationConfiguration* ui_;
  ConfigurableWidgetDialog* camera_transform_guess_dialog_;
  ConfigurableWidgetDialog* target_transform_guess_dialog_;
  ConfigurableWidgetDialog* camera_intrinsics_dialog_;
  std::map<QString, ConfigurableWidgetDialog*> target_dialogs_;
};

}  // namespace industrial_calibration
