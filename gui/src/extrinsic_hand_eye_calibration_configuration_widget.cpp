#include "ui_extrinsic_hand_eye_calibration_configuration_widget.h"
#include <industrial_calibration/core/serialization.h>
#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_configuration_widget.h>
#include <industrial_calibration/gui/transform_guess.h>
#include <industrial_calibration/gui/camera_intrinsics.h>
#include <industrial_calibration/gui/charuco_grid_target_finder.h>
#include <industrial_calibration/gui/aruco_grid_target_finder.h>
#include <industrial_calibration/gui/modified_circle_grid_target_finder.h>

#include <fstream>
#include <QFileDialog>
#include <QMessageBox>

namespace industrial_calibration
{
template <typename WidgetT>
ConfigurableWidgetDialog* setup(QWidget* const parent, QAbstractButton* const button = nullptr)
{
  auto* widget = new WidgetT(parent);
  auto* dialog = new ConfigurableWidgetDialog(widget, parent);

  // Optionally connect to button clicked signal
  if (button != nullptr)
  {
    QObject::connect(button, &QAbstractButton::clicked, dialog, &QWidget::show);
  }

  return dialog;
};

ExtrinsicHandEyeCalibrationConfigurationWidget::ExtrinsicHandEyeCalibrationConfigurationWidget(QWidget* parent)
  : QWidget(parent), ui_(new Ui::ExtrinsicHandEyeCalibrationConfiguration())
{
  ui_->setupUi(this);

  // Set up dialog boxes
  camera_transform_guess_dialog_ = setup<TransformGuess>(this, ui_->tool_button_camera_guess);
  target_transform_guess_dialog_ = setup<TransformGuess>(this, ui_->tool_button_target_guess);
  camera_intrinsics_dialog_ = setup<CameraIntrinsicsWidget>(this, ui_->tool_button_camera_intrinsics);

  target_dialogs_["CharucoGridTargetFinder"] = setup<CharucoGridTargetFinderWidget>(this);
  target_dialogs_["ArucoGridTargetFinder"] = setup<ArucoGridTargetFinderWidget>(this);
  target_dialogs_["ModifiedCircleGridTargetFinder"] = setup<ModifiedCircleGridTargetFinderWidget>(this);

  // Add the target finder names to the combo box
  for (auto& pair : target_dialogs_)
    ui_->combo_box_target_finder->addItem(pair.first);

  connect(ui_->tool_button_target_finder, &QAbstractButton::clicked, [this]() {
    QString type = ui_->combo_box_target_finder->currentText();
    target_dialogs_.at(type)->show();
  });

  connect(ui_->push_button_save, &QPushButton::clicked, this, &ExtrinsicHandEyeCalibrationConfigurationWidget::save);
}

ExtrinsicHandEyeCalibrationConfigurationWidget::~ExtrinsicHandEyeCalibrationConfigurationWidget() { delete ui_; }

void ExtrinsicHandEyeCalibrationConfigurationWidget::load(const QString& config_file)
{
  // Load all of the configurations before setting GUI items
  YAML::Node node = YAML::LoadFile(config_file.toStdString());
  auto intrinsics = getMember<YAML::Node>(node, "intrinsics");
  auto camera_mount_to_camera = getMember<YAML::Node>(node, "camera_mount_to_camera_guess");
  auto target_mount_to_target = getMember<YAML::Node>(node, "target_mount_to_target_guess");
  auto homography_threshold = getMember<double>(node, "homography_threshold");
  auto static_camera = getMember<bool>(node, "static_camera");
  auto target_finder_config = getMember<YAML::Node>(node, "target_finder");

  // Load parameters
  // Intrinsics
  camera_intrinsics_dialog_->widget->configure(intrinsics);

  // Guess transforms
  camera_transform_guess_dialog_->widget->configure(camera_mount_to_camera);
  target_transform_guess_dialog_->widget->configure(target_mount_to_target);

  // Homography
  ui_->double_spin_box_homography->setValue(homography_threshold);

  // Target
  QString target_type = QString::fromStdString(getMember<std::string>(target_finder_config, "type"));
  int idx = ui_->combo_box_target_finder->findText(target_type);
  if (idx < 0) throw std::runtime_error("Unknown target type '" + target_type.toStdString() + "'");
  target_dialogs_.at(target_type)->widget->configure(target_finder_config);
  ui_->combo_box_target_finder->setCurrentIndex(idx);

  ui_->check_box_static_camera->setChecked(static_camera);
}

void ExtrinsicHandEyeCalibrationConfigurationWidget::save()
{
  try
  {
    // Get filepath
    const QString file =
        QFileDialog::getSaveFileName(this, "Save calibration configuration", QString(), "YAML files (*.yaml *.yml)");

    if (file.isNull()) return;

    YAML::Node node;
    // Camera intrinsics
    node["intrinsics"] = camera_intrinsics_dialog_->widget->save();

    // Transform guesses
    node["camera_mount_to_camera_guess"] = camera_transform_guess_dialog_->widget->save();
    node["target_mount_to_target_guess"] = target_transform_guess_dialog_->widget->save();

    // Homography
    node["homography_threshold"] = ui_->double_spin_box_homography->value();

    // Target
    QString target_type = ui_->combo_box_target_finder->currentText();
    node["target_finder"] = target_dialogs_.at(target_type)->widget->save();

    node["static_camera"] = ui_->check_box_static_camera->isChecked();

    std::ofstream fout(file.toStdString());
    fout << node;
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

YAML::Node ExtrinsicHandEyeCalibrationConfigurationWidget::getTargetFinderConfig() const
{
  QString target_type = ui_->combo_box_target_finder->currentText();
  return target_dialogs_.at(target_type)->widget->save();
}

Eigen::Isometry3d ExtrinsicHandEyeCalibrationConfigurationWidget::getCameraMountToCameraGuess() const
{
  return camera_transform_guess_dialog_->widget->save().as<Eigen::Isometry3d>();
}

Eigen::Isometry3d ExtrinsicHandEyeCalibrationConfigurationWidget::getTargetMountToTargetGuess() const
{
  return target_transform_guess_dialog_->widget->save().as<Eigen::Isometry3d>();
}

CameraIntrinsics ExtrinsicHandEyeCalibrationConfigurationWidget::getCameraIntrinsics() const
{
  return camera_intrinsics_dialog_->widget->save().as<CameraIntrinsics>();
}

double ExtrinsicHandEyeCalibrationConfigurationWidget::getHomographyThreshold() const
{
  return ui_->double_spin_box_homography->value();
}

bool ExtrinsicHandEyeCalibrationConfigurationWidget::getStaticCamera() const
{
  return ui_->check_box_static_camera->isChecked();
}

}  // namespace industrial_calibration
