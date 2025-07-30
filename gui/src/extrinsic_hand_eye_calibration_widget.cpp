#include "ui_camera_calibration_data_manager_widget.h"

#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_widget.h>
#include <industrial_calibration/gui/target_finder.h>
#include <industrial_calibration/gui/camera_intrinsics.h>
#include <industrial_calibration/gui/transform_guess.h>
#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <industrial_calibration/target_finders/opencv/utils.h>
#include <industrial_calibration/core/serialization.h>
// Analysis
#include <industrial_calibration/analysis/projection.h>
#include <industrial_calibration/analysis/extrinsic_hand_eye_calibration_analysis.h>
#include <industrial_calibration/analysis/homography_analysis.h>

#include <fstream>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>

namespace industrial_calibration
{
ExtrinsicHandEyeCalibrationWidget::ExtrinsicHandEyeCalibrationWidget(QWidget* parent)
  : CameraCalibrationDataManagerWidget(parent)
  , camera_transform_guess_widget_(new TransformGuess(this))
  , target_transform_guess_widget_(new TransformGuess(this))
  , action_load_configuration(new QAction("Load configuration file...", this))
  , action_camera_mount_to_camera(new QAction("Camera mount to camera transform", this))
  , action_target_mount_to_target(new QAction("Target mount to target transform", this))
  , action_static_camera(new QAction("Static camera", this))
  , action_calibrate(new QAction("Calibrate", this))
  , action_save(new QAction("Save calibration...", this))
{
  // Load configuration
  action_load_configuration->setToolTip("Load extrinsic calibration configuration file...");
  action_load_configuration->setIcon(QIcon::fromTheme("document-properties"));
  connect(action_load_configuration, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::onLoadConfig);

  // Set up the camera mount to camera transform guess widget
  {
    auto dialog = new QDialog(this);
    auto layout = new QVBoxLayout(dialog);
    layout->addWidget(camera_transform_guess_widget_);
    dialog->setWindowTitle("Edit camera mount to camera transform guess");

    action_camera_mount_to_camera->setToolTip("Edit camera mount to camera transform guess");
    action_camera_mount_to_camera->setIcon(QIcon(":/icons/frame_camera.png"));
    connect(action_camera_mount_to_camera, &QAction::triggered, dialog, &QWidget::show);
  }

  // Set up the target mount to target transform guess widget
  {
    auto dialog = new QDialog(this);
    auto layout = new QVBoxLayout(dialog);
    layout->addWidget(target_transform_guess_widget_);
    dialog->setWindowTitle("Edit target mount to target transform guess");

    action_target_mount_to_target->setToolTip("Edit target mount to target transform guess");
    action_target_mount_to_target->setIcon(QIcon(":/icons/frame_target.png"));
    connect(action_target_mount_to_target, &QAction::triggered, dialog, &QWidget::show);
  }

  // Static camera action
  action_static_camera->setToolTip("Enable if camera is statically mounted");
  action_static_camera->setIcon(QIcon::fromTheme("user-available"));
  action_static_camera->setCheckable(true);

  // Calibration action
  action_calibrate->setToolTip("Run calibration");
  action_calibrate->setIcon(QIcon::fromTheme("media-playback-start"));
  connect(action_calibrate, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::onCalibrate);

  // Save action
  action_save->setToolTip("Save calibration results to file...");
  action_save->setIcon(QIcon::fromTheme("document-save"));
  connect(action_save, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::onSaveResults);
}

void ExtrinsicHandEyeCalibrationWidget::onLoadConfig()
{
  try
  {
    // Get yaml filepath
    const QString config_file = QFileDialog::getOpenFileName(this, "Load calibration configuration file", QString(),
                                                             "YAML files (*.yaml *.yml)");
    if (config_file.isNull() || config_file.isEmpty())
      return;

    loadConfig(config_file.toStdString());

    QMessageBox::information(this, "Configuration", "Successfully loaded calibration configuration");
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void ExtrinsicHandEyeCalibrationWidget::loadConfig(const std::string& config_file)
{
  // Load all of the configurations before setting GUI items
  YAML::Node node = YAML::LoadFile(config_file);
  auto target_finder_config = getMember<YAML::Node>(node, "target_finder");
  auto intrinsics = getMember<YAML::Node>(node, "intrinsics");
  auto camera_mount_to_camera = getMember<YAML::Node>(node, "camera_mount_to_camera_guess");
  auto target_mount_to_target = getMember<YAML::Node>(node, "target_mount_to_target_guess");
  auto homography_threshold = getMember<double>(node, "homography_threshold");
  auto static_camera = getMember<bool>(node, "static_camera");

  target_finder_widget_->configure(target_finder_config);
  camera_intrinsics_widget_->configure(intrinsics);
  camera_transform_guess_widget_->configure(camera_mount_to_camera);
  target_transform_guess_widget_->configure(target_mount_to_target);
  ui_->double_spin_box_homography_threshold->setValue(homography_threshold);
  action_static_camera->setChecked(static_camera);
}

void ExtrinsicHandEyeCalibrationWidget::onCalibrate()
{
  try
  {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    calibrate();
    QApplication::restoreOverrideCursor();

    if (result_->converged)
    {
      emit calibrationComplete(*result_);
      QMessageBox::information(this, "Calibration", "Successfully completed calibration");
    }
    else
      QMessageBox::warning(this, "Error", "Calibration failed to converge");

    // Change the tab widget to the results page
    ui_->tab_widget->setCurrentWidget(ui_->tab_results);
  }
  catch (const std::exception& ex)
  {
    result_ = nullptr;
    QApplication::restoreOverrideCursor();
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void ExtrinsicHandEyeCalibrationWidget::calibrate()
{
  // Check if there are any observations before continuing
  if (ui_->tree_widget_observations->topLevelItemCount() == 0)
    throw std::runtime_error("Please load the calibration observations before performing the calibration");

  // Load the target finder
  loadTargetFinder();

  // Create the calibration problem
  ExtrinsicHandEyeProblem2D3D problem;
  problem.camera_mount_to_camera_guess = camera_transform_guess_widget_->save().as<Eigen::Isometry3d>();
  problem.target_mount_to_target_guess = target_transform_guess_widget_->save().as<Eigen::Isometry3d>();
  problem.intr = camera_intrinsics_widget_->save().as<CameraIntrinsics>();

  for (int i = 0; i < ui_->tree_widget_observations->topLevelItemCount(); ++i)
  {
    // Extract the tree items
    QTreeWidgetItem* item = ui_->tree_widget_observations->topLevelItem(i);
    QTreeWidgetItem* features_item = item->child(IDX_FEATURES);
    QTreeWidgetItem* homography_item = item->child(IDX_HOMOGRAPHY);

    QString image_file = item->data(0, IMAGE_FILE_NAME_ROLE).value<QString>();
    if (!QFile(image_file).exists())
    {
      error(item, "Image file does not exist");
      continue;
    }

    QString pose_file = item->data(0, POSE_FILE_NAME_ROLE).value<QString>();
    if (!QFile(pose_file).exists())
    {
      error(item, "Pose file does not exist");
      continue;
    }

    try
    {
      // Load the image and pose
      cv::Mat image = readImageOpenCV(image_file.toStdString());
      auto pose = YAML::LoadFile(pose_file.toStdString()).as<Eigen::Isometry3d>();

      // Populate an observation
      Observation2D3D obs;
      if (action_static_camera->isChecked())
      {
        obs.to_camera_mount = Eigen::Isometry3d::Identity();
        obs.to_target_mount = pose;
      }
      else
      {
        obs.to_camera_mount = pose;
        obs.to_target_mount = Eigen::Isometry3d::Identity();
      }
      obs.correspondence_set = target_finder_->findCorrespondences(image);

      // Calculate homography error
      RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(), obs.correspondence_set.size() / 3,
                                                 RANDOM_SEED);
      Eigen::VectorXd homography_error = calculateHomographyError(obs.correspondence_set, random_sampler);
      double homography_error_mean = homography_error.array().mean();

      // Conditionally add the observation to the problem if the mean homography error is less than the threshold
      if (homography_error_mean < ui_->double_spin_box_homography_threshold->value())
      {
        problem.observations.push_back(obs);
        info(item, "Included in calibration");
      }
      else
      {
        // Update the notes
        error(item, "Excluded from calibration (homography threshold violation)");
      }

      // Update the tree widget
      info(features_item, QString::number(obs.correspondence_set.size()));
      info(homography_item, QString::number(homography_error_mean));
    }
    catch (const std::exception& ex)
    {
      error(item, QString(ex.what()));
    }
  }

  // Solve the calibration problem
  result_ = std::make_shared<ExtrinsicHandEyeResult>();
  *result_ = optimize(problem);

  // Report results
  std::stringstream ss;
  ss << *result_ << std::endl;
  ss << result_->covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

  // Compute the projected 3D error for comparison
  ss << analyze3dProjectionError(problem, *result_) << std::endl << std::endl;

  // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
  // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic
  // parameters We will then see how much this transform differs from the same transform calculated using the results
  // of the extrinsic calibration
  ExtrinsicHandEyeAnalysisStats stats = analyzeResults(problem, *result_);
  ss << stats << std::endl << std::endl;

  ui_->text_edit_results->clear();
  ui_->text_edit_results->append(QString::fromStdString(ss.str()));
}

void ExtrinsicHandEyeCalibrationWidget::onSaveResults()
{
  try
  {
    const QString file = QFileDialog::getSaveFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");
    if (file.isNull() || file.isEmpty())
      return;

    saveResults(file.toStdString());
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void ExtrinsicHandEyeCalibrationWidget::saveResults(const std::string& file)
{
  if (result_ == nullptr)
    throw ICalException("Calibration problem has not yet been solved. Please load the calibration data and run the "
                        "calibration");

  std::ofstream f(file);
  f << YAML::Node(*result_);
}

}  // namespace industrial_calibration
