#include "ui_camera_calibration_data_manager_widget.h"

#include <industrial_calibration/gui/camera_intrinsic_calibration_widget.h>
#include <industrial_calibration/gui/target_finder.h>
#include <industrial_calibration/gui/camera_intrinsics.h>
#include <industrial_calibration/analysis/statistics.h>
#include <industrial_calibration/analysis/camera_intrinsic_calibration_analysis.h>
#include <industrial_calibration/optimizations/camera_intrinsic.h>
#include <industrial_calibration/target_finders/opencv/utils.h>
#include <industrial_calibration/core/serialization.h>
// Analysis
#include <industrial_calibration/analysis/homography_analysis.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>

namespace industrial_calibration
{
static CameraIntrinsicResult optimizeOpenCV(const CameraIntrinsicProblem& params, const cv::Size& image_size)
{
  std::vector<std::vector<cv::Vec3f>> object_points;
  std::vector<std::vector<cv::Vec2f>> image_points;

  for (const auto& o : params.image_observations)
  {
    std::vector<cv::Vec3f> op;
    std::vector<cv::Vec2f> ip;

    for (const auto& pair : o)
    {
      op.push_back(cv::Vec3f(pair.in_target(0), pair.in_target(1), pair.in_target(2)));
      ip.push_back(cv::Vec2f(pair.in_image(0), pair.in_image(1)));
    }

    object_points.push_back(op);
    image_points.push_back(ip);
  }

  // Convert intrinsics to OpenCV
  cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
  cv::setIdentity(camera_matrix);
  camera_matrix.at<double>(0, 0) = params.intrinsics_guess.fx();
  camera_matrix.at<double>(1, 1) = params.intrinsics_guess.fy();
  camera_matrix.at<double>(0, 2) = params.intrinsics_guess.cx();
  camera_matrix.at<double>(1, 2) = params.intrinsics_guess.cy();

  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  double rms_error =
      cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

  // Populate the result
  CameraIntrinsicResult result;
  result.converged = true;
  result.initial_cost_per_obs = std::numeric_limits<double>::infinity();
  result.final_cost_per_obs = rms_error;

  // Intrinsics
  result.intrinsics.fx() = camera_matrix.at<double>(0, 0);
  result.intrinsics.fy() = camera_matrix.at<double>(1, 1);
  result.intrinsics.cx() = camera_matrix.at<double>(0, 2);
  result.intrinsics.cy() = camera_matrix.at<double>(1, 2);

  // Distortion
  result.distortions[0] = dist_coeffs.at<double>(0);
  result.distortions[1] = dist_coeffs.at<double>(1);
  result.distortions[2] = dist_coeffs.at<double>(2);
  result.distortions[3] = dist_coeffs.at<double>(3);
  result.distortions[4] = dist_coeffs.at<double>(4);

  // Target positions
  result.target_transforms.reserve(params.image_observations.size());
  for (std::size_t i = 0; i < params.image_observations.size(); ++i)
  {
    // Convert to rotation matrix
    cv::Mat rotation(3, 3, CV_64F);
    cv::Rodrigues(rvecs[i], rotation);

    Eigen::Isometry3d pose;
    pose.linear() = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Map(reinterpret_cast<double*>(rotation.data));
    pose.translation() = Eigen::Vector3d::Map(reinterpret_cast<double*>(tvecs[i].data));

    result.target_transforms.push_back(pose);
  }

  return result;
}

CameraIntrinsicCalibrationWidget::CameraIntrinsicCalibrationWidget(QWidget* parent)
  : CameraCalibrationDataManagerWidget(parent)
  , action_load_configuration(new QAction("Load configuration file...", this))
  , action_use_extrinsic_guesses(new QAction("Use target pose guesses", this))
  , action_use_opencv(new QAction("Use OpenCV calibration", this))
  , action_calibrate(new QAction("Calibrate", this))
  , action_save(new QAction("Save calibration...", this))
  , action_save_ros_format(new QAction("Save calibration (ROS)...", this))
{
  // Load configuration file
  action_load_configuration->setToolTip("Load intrinsic calibration configuration file...");
  action_load_configuration->setIcon(QIcon::fromTheme("document-properties"));
  connect(action_load_configuration, &QAction::triggered, this, &CameraIntrinsicCalibrationWidget::onLoadConfig);

  // Use target pose guesses flag
  action_use_extrinsic_guesses->setToolTip("Enable to use target pose guesses in optimization (recommended)");
  action_use_extrinsic_guesses->setIcon(QIcon::fromTheme("user-available"));
  action_use_extrinsic_guesses->setCheckable(true);

  // Use OpenCV calibration flag
  action_use_opencv->setToolTip("Enable to use OpenCV calibration algorithm");
  action_use_opencv->setIcon(QIcon(":icons/opencv.svg"));
  action_use_opencv->setCheckable(true);

  // Calibrate
  action_calibrate->setToolTip("Run calibration");
  action_calibrate->setIcon(QIcon::fromTheme("media-playback-start"));
  connect(action_calibrate, &QAction::triggered, this, &CameraIntrinsicCalibrationWidget::onCalibrate);

  // Save
  action_save->setToolTip("Save calibration results to file...");
  action_save->setIcon(QIcon::fromTheme("document-save"));
  connect(action_save, &QAction::triggered, this, &CameraIntrinsicCalibrationWidget::onSaveResults);

  // Save ROS format
  std::string tool_tip =
      R"(
          <html><head/><body><p>Save the calibration results to a <a href="https://wiki.ros.org/camera_calibration_parsers#File_formats"><span style=" text-decoration: underline; color:#007af4;">format comptible with ROS</span></a></p></body></html>
          )";
  action_save_ros_format->setToolTip(QString::fromStdString(tool_tip));
  action_save_ros_format->setIcon(QIcon::fromTheme("document-save-as"));
  connect(action_save_ros_format, &QAction::triggered, this, &CameraIntrinsicCalibrationWidget::onSaveROSFormat);
}

void CameraIntrinsicCalibrationWidget::onLoadConfig()
{
  try
  {
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

void CameraIntrinsicCalibrationWidget::loadConfig(const std::string& config_file)
{
  YAML::Node node = YAML::LoadFile(config_file);

  // Attempt to load all required configuraiton information before setting the GUI
  auto target_finder_config = getMember<YAML::Node>(node, "target_finder");
  YAML::Node intrinsics_config;
  try
  {
    intrinsics_config = getMember<YAML::Node>(node, "intrinsics_guess");
  }
  catch (const ICalException&)
  {
    intrinsics_config = getMember<YAML::Node>(node, "intrinsics");
  }
  const auto homography_threshold = getMember<double>(node, "homography_threshold");
  const bool use_extrinsic_guesses = getMember<bool>(node, "use_extrinsic_guesses");

  // Configure the GUI
  target_finder_widget_->configure(target_finder_config);
  camera_intrinsics_widget_->configure(intrinsics_config);
  ui_->double_spin_box_homography_threshold->setValue(homography_threshold);
  action_use_extrinsic_guesses->setChecked(use_extrinsic_guesses);

  // Load the optional parameters
  try
  {
    action_use_opencv->setChecked(getMember<bool>(node, "use_opencv"));
  }
  catch (const std::exception&)
  {
  }
}

void CameraIntrinsicCalibrationWidget::onCalibrate()
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

void CameraIntrinsicCalibrationWidget::calibrate()
{
  // Check if there are any observations before continuing
  if (ui_->tree_widget_observations->topLevelItemCount() == 0)
    throw std::runtime_error("Please load the calibration observations before performing the calibration");

  // Load the target finder
  loadTargetFinder();

  // Create the calibration problem
  CameraIntrinsicProblem problem;
  problem.intrinsics_guess = camera_intrinsics_widget_->save().as<CameraIntrinsics>();
  problem.use_extrinsic_guesses = action_use_extrinsic_guesses->isChecked();

  // Extract the image size for the OpenCV solver
  cv::Size image_size;

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
    if (!QFile(pose_file).exists() && problem.use_extrinsic_guesses)
    {
      error(item, "Pose file does not exist");
      continue;
    }

    try
    {
      // Load the image and pose
      cv::Mat image = readImageOpenCV(image_file.toStdString());
      auto pose = YAML::LoadFile(pose_file.toStdString()).as<Eigen::Isometry3d>();

      if (i == 0)
        image_size = image.size();

      Correspondence2D3D::Set correspondence_set = target_finder_->findCorrespondences(image);

      // Calculate homography error
      RandomCorrespondenceSampler random_sampler(correspondence_set.size(), correspondence_set.size() / 3, RANDOM_SEED);
      Eigen::VectorXd homography_error = calculateHomographyError(correspondence_set, random_sampler);
      double homography_error_mean = homography_error.array().mean();

      // Conditionally add the observation to the problem if the mean homography error is less than the threshold
      if (homography_error_mean < ui_->double_spin_box_homography_threshold->value())
      {
        problem.image_observations.push_back(correspondence_set);
        if (problem.use_extrinsic_guesses)
          problem.extrinsic_guesses.push_back(pose);

        info(item, "Included in calibration");
      }
      else
      {
        // Update the notes
        error(item, "Excluded from calibration (homography threshold violation)");
      }

      // Update the tree widget
      info(features_item, QString::number(correspondence_set.size()));
      info(homography_item, QString::number(homography_error_mean));
    }
    catch (const std::exception& ex)
    {
      error(item, QString(ex.what()));
    }
  }

  // Solve the calibration problem
  result_ = std::make_shared<CameraIntrinsicResult>();
  if (action_use_opencv->isChecked())
    *result_ = optimizeOpenCV(problem, image_size);
  else
    *result_ = optimize(problem);

  // Perform analysis
  IntrinsicCalibrationAccuracyResult accuracy_result;
  {
    // Create accumulators for mean and variance
    std::vector<double> pos_acc, ang_acc;
    pos_acc.reserve(problem.image_observations.size());
    ang_acc.reserve(problem.image_observations.size());

    for (std::size_t i = 0; i < problem.image_observations.size(); ++i)
    {
      VirtualCorrespondenceResult vcr = measureVirtualTargetDiff(problem.image_observations[i], result_->intrinsics,
                                                                 result_->target_transforms[i], 1.0);
      pos_acc.push_back(vcr.positional_error);
      ang_acc.push_back(vcr.angular_error);
    }

    std::tie(accuracy_result.pos_error.first, accuracy_result.pos_error.second) = computeStats(pos_acc);
    std::tie(accuracy_result.ang_error.first, accuracy_result.ang_error.second) = computeStats(ang_acc);
  }

  // Report results
  std::stringstream ss;
  ss << *result_ << std::endl;
  ss << "\n" << std::setprecision(4) << accuracy_result << std::endl;
  if (result_->covariance.covariances.empty() && !action_use_opencv->isChecked())
    ss << "Failed to compute covariance" << std::endl;
  else if (!action_use_opencv->isChecked())
    ss << result_->covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

  ui_->text_edit_results->clear();
  ui_->text_edit_results->append(QString::fromStdString(ss.str()));
}

void CameraIntrinsicCalibrationWidget::onSaveResults()
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

void CameraIntrinsicCalibrationWidget::saveResults(const std::string& file) const
{
  if (result_ == nullptr)
    throw ICalException("Calibration problem has not yet been solved. Please load the calibration data and run the "
                        "calibration");

  std::ofstream f(file);
  f << YAML::Node(*result_);
}

void CameraIntrinsicCalibrationWidget::onSaveROSFormat()
{
  try
  {
    const QString file = QFileDialog::getSaveFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");
    if (file.isNull() || file.isEmpty())
      return;

    saveROSFormat(file.toStdString());
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void CameraIntrinsicCalibrationWidget::saveROSFormat(const std::string& file) const
{
  if (result_ == nullptr)
    return;

  // Image size
  if (ui_->tree_widget_observations->topLevelItemCount() == 0)
    throw ICalException("No observations have been added");

  QTreeWidgetItem* item = ui_->tree_widget_observations->topLevelItem(0);
  QString image_file = item->data(0, IMAGE_FILE_NAME_ROLE).value<QString>();
  if (!QFile(image_file).exists())
    throw ICalException("Image '" + image_file.toStdString() + "' does not exist");

  cv::Mat image = readImageOpenCV(image_file.toStdString());
  const int image_height = image.size[0];
  const int image_width = image.size[1];

  YAML::Node node = toROSFormat(*result_, image_width, image_height);
  std::ofstream f(file);
  f << node;
}

}  // namespace industrial_calibration
