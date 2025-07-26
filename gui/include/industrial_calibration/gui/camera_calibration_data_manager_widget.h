#pragma once

#include <industrial_calibration/target_finders/opencv/target_finder.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>
#include <QWidget>
#include <QDialog>

class QTreeWidget;
class QTreeWidgetItem;

namespace Ui
{
class CameraCalibrationDataManager;
}

namespace industrial_calibration
{
class TargetFinderWidget;
class CameraIntrinsicsWidget;

/**
 * @brief Widget for loading and managing a 2D image calibration data set.
 * @details This widget loads a 2D image calibration data set from a YAML file.
 * The calibration data set is stored internally in the widget in a `QTreeWidget`.
 * Each observation in the data set is stored as a `QTreeWidgetItem`, where the image file name is stored as data under
 * the @ref CameraCalibrationDataManagerWidget::IMAGE_FILE_NAME_ROLE role and the pose file name is stored as data under
 * the @ref CameraCalibrationDataManagerWidget::POSE_FILE_NAME_ROLE role. The observation `QTreeWidgetItem` has two
 * child items:
 *   - The number of features detected in the target (child item at index @ref
 * CameraCalibrationDataManagerWidget::IDX_FEATURES)
 *   - The homography error of the detected target (child item at index @ref
 * CameraCalibrationDataManagerWidget::IDX_HOMOGRAPHY)
 *
 * The TargetFinder used for calibration and CameraIntrinsics must be also be configured in the widget in order to
 * detect the calibration target in the images.
 *
 * When an image is selected, this widget attempts to detect the Target in the image using the configured
 * TargetDetector. If the target is found. the widget emits the @ref CameraCalibrationDataManagerWidget::imageSelected
 * signal with an image where the target features are drawn on the image. Otherwise, the widget emits the @ref
 * CameraCalibrationDataManagerWidget::imageSelected signal with the original observation image.
 */
class CameraCalibrationDataManagerWidget : public QWidget
{
  Q_OBJECT

public:
  explicit CameraCalibrationDataManagerWidget(QWidget* parent = nullptr);
  virtual ~CameraCalibrationDataManagerWidget();

  /**
   * @brief Loads the calibration observations from file (defined in @ref s_extrinsic_hand_eye_obs_def for extrinsic
   * hand eye calibration and @ref s_camera_intrnisic_obs_def for camera intrinsic calibration)
   * @throws Exception on failure
   */
  void loadObservations(const std::string& observations_file);

  QAction* action_load_observations;
  QAction* action_edit_target_finder;
  QAction* action_edit_camera_intrinsics;
  QTreeWidget* getTreeWidget() const;

signals:
  /**
   * @brief Signal emitted when an observation is selected in the tree widget.
   * If the target can be identified in the image associated with the observation, the signal provides that image with
   * the detected target drawn on it. Otherwise, the signal provides the original observation image.
   */
  void imageSelected(const QPixmap& image);

protected:
  /** @brief Random seed used for homography analysis */
  static const unsigned RANDOM_SEED;
  /** @brief Role for image file in tree widget */
  static const int IMAGE_FILE_NAME_ROLE;
  /** @brief Role for pose file in tree widget */
  static const int POSE_FILE_NAME_ROLE;
  /** @brief Data index of the detected target feature count inside a tree widget item */
  static const int IDX_FEATURES;
  /** @brief Data index of the homography error inside a tree widget item */
  static const int IDX_HOMOGRAPHY;

  /** @brief helper function for converting an OpenCV image to a Qt pixmap */
  static QPixmap toQt(const cv::Mat& image);

  /** @brief helper function for showing an info message with a tree item */
  static void info(QTreeWidgetItem* item, const QString& message);

  /** @brief helper function for showing an error message with a tree item */
  static void error(QTreeWidgetItem* item, const QString& message);

  void closeEvent(QCloseEvent* event) override;

  void onLoadObservations();
  void loadTargetFinder();
  void drawImage(QTreeWidgetItem* item, int col);

  Ui::CameraCalibrationDataManager* ui_;
  TargetFinderWidget* target_finder_widget_;
  CameraIntrinsicsWidget* camera_intrinsics_widget_;

  boost_plugin_loader::PluginLoader loader_;
  TargetFinderFactoryOpenCV::ConstPtr factory_;
  TargetFinderOpenCV::ConstPtr target_finder_;
};

}  // namespace industrial_calibration
