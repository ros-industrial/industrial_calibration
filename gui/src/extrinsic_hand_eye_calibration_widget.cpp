#include "ui_extrinsic_hand_eye_calibration_widget.h"

#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_widget.h>
#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_configuration_widget.h>
#include <industrial_calibration/gui/aspect_ratio_pixmap_label.h>
#include <industrial_calibration/optimizations/extrinsic_hand_eye.h>
#include <industrial_calibration/target_finders/opencv/utils.h>
#include <industrial_calibration/core/serialization.h>

// Analysis
#include <industrial_calibration/analysis/projection.h>
#include <industrial_calibration/analysis/extrinsic_hand_eye_calibration_analysis.h>
#include <industrial_calibration/analysis/homography_analysis.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <QDialog>
#include <QScrollBar>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QPixmap>
#include <QVBoxLayout>

static const unsigned RANDOM_SEED = 1;
static const int IMAGE_FILE_NAME_ROLE = Qt::UserRole + 1;
static const int POSE_FILE_NAME_ROLE = Qt::UserRole + 2;
static const int IDX_FEATURES = 0;
static const int IDX_HOMOGRAPHY = 1;

namespace industrial_calibration
{
QPixmap toQt(const cv::Mat& image)
{
  return QPixmap::fromImage(QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888).rgbSwapped());
}

/** @brief helper function for showing an info message with a tree item */
void info(QTreeWidgetItem* item, const QString& message)
{
  item->setText(1, message);
  item->setForeground(1, QApplication::palette().text());
}

/** @brief helper function for showing an error message with a tree item */
void error(QTreeWidgetItem* item, const QString& message)
{
  item->setText(1, message);
  item->setForeground(1, QBrush(QColor("red")));
}

ExtrinsicHandEyeCalibrationWidget::ExtrinsicHandEyeCalibrationWidget(QWidget* parent)
  : QMainWindow(parent)
  , ui_(new Ui::ExtrinsicHandEyeCalibration())
  , configuration_widget_(new ExtrinsicHandEyeCalibrationConfigurationWidget(this))
{
  ui_->setupUi(this);

  // Configure the tree widget
  ui_->tree_widget_observations->setEditTriggers(QAbstractItemView::NoEditTriggers);

  // Set up the configuration widget
  {
    auto dialog = new QDialog(this);
    auto layout = new QVBoxLayout(dialog);
    layout->addWidget(configuration_widget_);
    dialog->setWindowTitle("Configuration Settings");

    connect(ui_->action_edit_configuration, &QAction::triggered, dialog, &QWidget::show);
  }

  // Set the stretch factors of the horizontal splitter to make the proportions reasonable
  ui_->splitter_horizontal->setStretchFactor(0, 1);
  ui_->splitter_horizontal->setStretchFactor(1, 30);

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_results->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_results->verticalScrollBar()->setSliderPosition(
        ui_->text_edit_results->verticalScrollBar()->maximum());
  });

  // Set up push buttons
  connect(ui_->action_load_configuration, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::loadConfig);
  connect(ui_->action_calibrate, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::calibrate);
  connect(ui_->action_save, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::saveResults);

  connect(ui_->action_load_data, &QAction::triggered, this, &ExtrinsicHandEyeCalibrationWidget::loadObservations);
  connect(ui_->tree_widget_observations, &QTreeWidget::itemClicked, this,
          &ExtrinsicHandEyeCalibrationWidget::drawImage);

  // Set up the plugin loader
  loader_.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader_.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;
}

ExtrinsicHandEyeCalibrationWidget::~ExtrinsicHandEyeCalibrationWidget() { delete ui_; }

void ExtrinsicHandEyeCalibrationWidget::loadConfig()
{
  try
  {
    // Get yaml filepath
    const QString config_file = QFileDialog::getOpenFileName(this, "Load calibration configuration file", QString(),
                                                             "YAML files (*.yaml *.yml)");
    if (config_file.isNull()) return;

    configuration_widget_->load(config_file);

    QMessageBox::information(this, "Success", "Successfully loaded calibration configuration");
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void ExtrinsicHandEyeCalibrationWidget::loadTargetFinder()
{
  // Get target type and currentconfig
  YAML::Node target_finder_config = configuration_widget_->getTargetFinderConfig();
  auto target_type = getMember<std::string>(target_finder_config, "type");

  factory_ = loader_.createInstance<TargetFinderFactoryOpenCV>(target_type);
  target_finder_ = factory_->create(target_finder_config);
}

void ExtrinsicHandEyeCalibrationWidget::loadObservations()
{
  QString observations_file =
      QFileDialog::getOpenFileName(this, "Load calibration observation file", QString(), "YAML files (*.yaml *.yml)");
  if (observations_file.isNull()) return;

  QFileInfo observations_file_info(observations_file);

  try
  {
    auto observations = getMember<YAML::Node>(YAML::LoadFile(observations_file.toStdString()), "data");

    // Reset the tree widget
    ui_->tree_widget_observations->clear();

    for (std::size_t i = 0; i < observations.size(); ++i)
    {
      const YAML::Node& entry = observations[i];

      QString image_file =
          observations_file_info.absoluteDir().filePath(QString::fromStdString(getMember<std::string>(entry, "image")));
      QString pose_file =
          observations_file_info.absoluteDir().filePath(QString::fromStdString(getMember<std::string>(entry, "pose")));

      auto item = new QTreeWidgetItem();
      item->setData(0, Qt::EditRole, "Observation " + QString::number(i));
      item->setData(0, IMAGE_FILE_NAME_ROLE, image_file);
      item->setData(0, POSE_FILE_NAME_ROLE, pose_file);

      // Add a column entry for the number of detected features
      auto features_item = new QTreeWidgetItem(item);
      features_item->setData(0, Qt::EditRole, "Feature count");
      info(features_item, "-");

      // Add a column entry for the homography error
      auto homography_item = new QTreeWidgetItem(item);
      homography_item->setData(0, Qt::EditRole, "Homography error (px)");
      info(homography_item, "-");

      ui_->tree_widget_observations->addTopLevelItem(item);
    }

    ui_->tree_widget_observations->resizeColumnToContents(0);
  }
  catch (const std::exception& ex)
  {
    ui_->tree_widget_observations->clear();
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void ExtrinsicHandEyeCalibrationWidget::drawImage(QTreeWidgetItem* item, int col)
{
  if (item == nullptr) return;

  // Extract the top level item
  while (item->parent() != nullptr)
    item = item->parent();

  QTreeWidgetItem* features_item = item->child(IDX_FEATURES);
  QTreeWidgetItem* homography_item = item->child(IDX_HOMOGRAPHY);

  QString image_file = item->data(0, IMAGE_FILE_NAME_ROLE).value<QString>();
  if (!QFile(image_file).exists())
  {
    error(item, "Image file does not exist");
    return;
  }

  try
  {
    loadTargetFinder();
    cv::Mat image = cv::imread(image_file.toStdString());

    // Detect the target in the image
    TargetFeatures2D features = target_finder_->findTargetFeatures(image);
    cv::Mat detected_image = target_finder_->drawTargetFeatures(image, features);
    info(item, "Successfully identified target");

    // Save the number of detected features to the tree
    info(features_item, QString::number(features.size()));

    // Set the image
    ui_->image_label->setPixmap(toQt(detected_image));
    update();

    // Compute the homography error
    Correspondence2D3D::Set corrs = target_finder_->target().createCorrespondences(features);
    RandomCorrespondenceSampler random_sampler(corrs.size(), corrs.size() / 3, RANDOM_SEED);
    Eigen::VectorXd homography_error = calculateHomographyError(corrs, random_sampler);
    double homography_error_mean = homography_error.array().mean();

    // Save the homography error to the tree
    info(homography_item, QString::number(homography_error_mean));

    // Check homography threshold and update notes/row color
    if (homography_error_mean > configuration_widget_->getHomographyThreshold())
      error(item, "Homography threshold violated");
  }
  catch (const std::exception& ex)
  {
    cv::Mat image = cv::imread(image_file.toStdString());
    ui_->image_label->setPixmap(toQt(image));
    update();

    error(item, QString(ex.what()));
    info(features_item, "-");
    info(homography_item, "-");
  }
}

void ExtrinsicHandEyeCalibrationWidget::calibrate()
{
  try
  {
    QApplication::setOverrideCursor(Qt::WaitCursor);

    // Check if there are any observations before continuing
    if (ui_->tree_widget_observations->topLevelItemCount() == 0)
      throw std::runtime_error("Please load the calibration observations before performing the calibration");

    // Load the target finder
    loadTargetFinder();

    // Create the calibration problem
    ExtrinsicHandEyeProblem2D3D problem;
    problem.camera_mount_to_camera_guess = configuration_widget_->getCameraMountToCameraGuess();
    problem.target_mount_to_target_guess = configuration_widget_->getTargetMountToTargetGuess();
    problem.intr = configuration_widget_->getCameraIntrinsics();

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
      if (!QFile(image_file).exists())
      {
        error(item, "Pose file does not exist");
        continue;
      }

      try
      {
        // Load the image and pose
        cv::Mat image = cv::imread(image_file.toStdString());
        auto pose = YAML::LoadFile(pose_file.toStdString()).as<Eigen::Isometry3d>();

        // Populate an observation
        Observation2D3D obs;
        if (configuration_widget_->getStaticCamera())
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
        if (homography_error_mean < configuration_widget_->getHomographyThreshold())
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

    QApplication::restoreOverrideCursor();
    if (result_->converged)
      QMessageBox::information(this, "Success", "Successfully completed calibration");
    else
      QMessageBox::warning(this, "Error", "Calibration failed to converge");
  }
  catch (const std::exception& ex)
  {
    result_ = nullptr;
    QApplication::restoreOverrideCursor();
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void ExtrinsicHandEyeCalibrationWidget::saveResults()
{
  if (result_ == nullptr)
  {
    QMessageBox::warning(this, "Error",
                         "Calibration problem has not yet been solved. Please load the calibration data and run the "
                         "calibration");
    return;
  }

  const QString file = QFileDialog::getSaveFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");
  std::ofstream f(file.toStdString());
  f << YAML::Node(*result_);
}

}  // namespace industrial_calibration
