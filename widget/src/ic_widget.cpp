#include "widget/ic_widget.h"
#include "widget/transform_guess.h"
#include "widget/camera_intrinsics.h"
#include "widget/charuco_grid_target_finder.h"
#include "widget/aruco_grid_target_finder.h"
#include "widget/modified_circle_grid_target_finder.h"
#include "widget/image_widget.h"
#include "ui_ic_widget.h"
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
#include <QLabel>
#include <QImageReader>
#include <QMessageBox>
#include <QPixmap>
#include <QVBoxLayout>

static const unsigned RANDOM_SEED = 1;
static const int IMAGE_FILE_NAME_ROLE = Qt::UserRole + 1;
static const int POSE_FILE_NAME_ROLE = Qt::UserRole + 2;
static const int COL_FEATURES = 0;
static const int COL_HOMOGRAPHY = 1;
static const int COL_NOTES = 2;

ICDialog::ICDialog(ConfigurableWidget* widget_, QWidget* parent) : QDialog(parent), widget(widget_)
{
  auto* vl = new QVBoxLayout(this);
  vl->addWidget(widget);
  setWindowTitle("");
}

template<typename WidgetT>
ICDialog* setup(QWidget* const parent, QAbstractButton* const button = nullptr)
{
  auto* widget = new WidgetT(parent);
  auto* dialog = new ICDialog(widget, parent);

  // Optionally connect to button clicked signal
  if (button != nullptr)
  {
    QObject::connect(button, &QAbstractButton::clicked, dialog, &QWidget::show);
  }

  return dialog;
};

QPixmap toQt(const cv::Mat& image)
{
  return QPixmap::fromImage(QImage(image.data,
                                   image.cols,
                                   image.rows,
                                   image.step,
                                   QImage::Format_RGB888).rgbSwapped());
}

namespace industrial_calibration
{
ICWidget::ICWidget(QWidget *parent) :
    QWidget(parent),
    ui_(new Ui::ICWidget)
{
    ui_->setupUi(this);

    // Configure the table widget
    ui_->table_widget_data->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui_->table_widget_data->resizeColumnsToContents();
    ui_->table_widget_data->horizontalHeader()->stretchLastSection();

    // Set up dialog boxes
    camera_transform_guess_dialog_ = setup<TransformGuess>(this, ui_->tool_button_camera_guess);
    target_transform_guess_dialog_ = setup<TransformGuess>(this, ui_->tool_button_target_guess);
    camera_intrinsics_dialog_ = setup<CameraIntrinsicsWidget>(this, ui_->tool_button_camera_intrinsics);
    
    target_dialogs_["CharucoGridTargetFinder"] = setup<CharucoGridTargetFinderWidget>(this);
    target_dialogs_["ArucoGridTargetFinder"] = setup<ArucoGridTargetFinderWidget>(this);
    target_dialogs_["ModifiedCircleGridTargetFinder"] = setup<ModifiedCircleGridTargetFinderWidget>(this);

    // Move the text edit scroll bar to the maximum limit whenever it is resized
    connect(ui_->text_edit_log->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
        ui_->text_edit_log->verticalScrollBar()->setSliderPosition(ui_->text_edit_log->verticalScrollBar()->maximum());
    });

    // Set up push buttons
    connect(ui_->push_button_load_config, &QPushButton::clicked, this, &ICWidget::loadConfig);
    connect(ui_->push_button_save_config, &QPushButton::clicked, this, &ICWidget::saveConfig);
    connect(ui_->push_button_calibrate, &QPushButton::clicked, this, &ICWidget::calibrate);
    connect(ui_->push_button_save, &QPushButton::clicked, this, &ICWidget::saveResults);

    connect(ui_->tool_button_target_finder, &QAbstractButton::clicked, [this](){
        QString type = ui_->combo_box_target_finder->currentText();
        target_dialogs_.at(type)->show();
    });

    connect(ui_->push_button_load_data, &QAbstractButton::clicked, this, &ICWidget::loadData);
    connect(ui_->table_widget_data, &QTableWidget::cellPressed, this, &ICWidget::drawImage);

    // Set up the plugin loader
    loader_.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
    loader_.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;
}

ICWidget::~ICWidget()
{
    delete ui_;
}

void ICWidget::loadConfig()
{
    try
    {
        // Get yaml filepath
        const QString config_file = QFileDialog::getOpenFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");
        if (config_file.isNull())
            return;

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
        if (idx < 0)
            throw std::runtime_error("Unknown target type '" + target_type.toStdString() + "'");
        target_dialogs_.at(target_type)->widget->configure(target_finder_config);
        ui_->combo_box_target_finder->setCurrentIndex(idx);

        // Sensor configuration (historically industrical calibration yaml files don't include this)
        ui_->check_box_static_camera->setChecked(static_camera);

        ui_->line_edit_config->setText(config_file);
        QMessageBox::information(this, "Success", "Successfully loaded calibration configuration");
    }
    catch(const std::exception& ex)
    {
        QMessageBox::warning(this, "Error", ex.what());
    }
}

void ICWidget::saveConfig()
{
    // Get filepath
    const QString file = QFileDialog::getSaveFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");

    if (file.isNull())
        return;

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

void ICWidget::loadTargetFinder()
{
    // Get target type and currentconfig
    QString target_type = ui_->combo_box_target_finder->currentText();
    YAML::Node target_finder_config = target_dialogs_.at(target_type)->widget->save();

    factory_ = loader_.createInstance<TargetFinderFactoryOpenCV>(target_type.toStdString());
    target_finder_ = factory_->create(target_finder_config);
}

void ICWidget::loadData()
{
    QString data_file = QFileDialog::getOpenFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");
    if (data_file.isNull())
        return;

    QFileInfo data_file_info(data_file);

    try
    {
        auto data = getMember<YAML::Node>(YAML::LoadFile(data_file.toStdString()), "data");

        // Reset the table widget
        ui_->table_widget_data->clearContents();
        ui_->table_widget_data->setRowCount(data.size());

        for(std::size_t i = 0; i < data.size(); ++i)
        {
            const YAML::Node& entry = data[i];

            QString image_file = data_file_info.absoluteDir().filePath(QString::fromStdString(getMember<std::string>(entry, "image")));
            QString pose_file = data_file_info.absoluteDir().filePath(QString::fromStdString(getMember<std::string>(entry, "pose")));

            // Add a column entry for the number of detected features
            auto features_item = new QTableWidgetItem("-");
            features_item->setData(IMAGE_FILE_NAME_ROLE, image_file);
            features_item->setData(POSE_FILE_NAME_ROLE, pose_file);
            ui_->table_widget_data->setItem(i, COL_FEATURES, features_item);

            // Add a column entry for the homography error
            auto homography_item = new QTableWidgetItem("-");
            ui_->table_widget_data->setItem(i, COL_HOMOGRAPHY, homography_item);

            // Add a column entry for notes
            auto notes_item = new QTableWidgetItem("");
            ui_->table_widget_data->setItem(i, COL_NOTES, notes_item);
        }

        ui_->line_edit_data->setText(data_file);
        ui_->table_widget_data->resizeColumnsToContents();
        ui_->table_widget_data->horizontalHeader()->stretchLastSection();
    }
    catch(const std::exception& ex)
    {
        ui_->table_widget_data->clearContents();
        QMessageBox::warning(this, "Error", ex.what());
    }
}

void ICWidget::drawImage(int row, int col)
{
    QTableWidgetItem* features_item = ui_->table_widget_data->item(row, COL_FEATURES);
    QTableWidgetItem* homography_item = ui_->table_widget_data->item(row, COL_HOMOGRAPHY);
    QTableWidgetItem* notes_item = ui_->table_widget_data->item(row, COL_NOTES);

    if (features_item == nullptr)
        return;

    QVariant data = features_item->data(IMAGE_FILE_NAME_ROLE);
    if(data.isNull() || !data.canConvert<QString>())
        return;

    QString image_file = features_item->data(IMAGE_FILE_NAME_ROLE).value<QString>();
    if(!QFile(image_file).exists())
    {
        notes_item->setData(Qt::EditRole, "Image file does not exist");
        return;
    }

    try
    {
        loadTargetFinder();
        cv::Mat image = cv::imread(image_file.toStdString());

        // Attempt to detect the target in the image
        TargetFeatures2D features = target_finder_->findTargetFeatures(image);
        cv::Mat detected_image = target_finder_->drawTargetFeatures(image, features);

        // Save the number of detected features to the table
        features_item->setData(Qt::EditRole, QString::number(features.size()));

        // Set the image
        ui_->image_label->setPixmap(toQt(detected_image));
        update();

        // Attempt to compute the homography error
        Correspondence2D3D::Set corrs = target_finder_->target().createCorrespondences(features);
        RandomCorrespondenceSampler random_sampler(corrs.size(), corrs.size() / 3, RANDOM_SEED);
        Eigen::VectorXd homography_error = calculateHomographyError(corrs, random_sampler);
        double homography_error_mean = homography_error.array().mean();

        // Save the homography error to the table
        homography_item->setData(Qt::EditRole, QString::number(homography_error_mean));

        // Update the notes
        notes_item->setData(Qt::EditRole, "");
    }
    catch(const std::exception& ex)
    {
        cv::Mat image = cv::imread(image_file.toStdString());
        ui_->image_label->setPixmap(toQt(image));
        update();

        features_item->setData(Qt::EditRole, "-");
        homography_item->setData(Qt::EditRole, "-");
        notes_item->setData(Qt::EditRole, ex.what());
    }
}

void ICWidget::calibrate()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);

    try
    {
        // Load the target finder
        loadTargetFinder();

        // Create the calibration problem
        ExtrinsicHandEyeProblem2D3D problem;
        problem.camera_mount_to_camera_guess = camera_transform_guess_dialog_->widget->save().as<Eigen::Isometry3d>();
        problem.target_mount_to_target_guess = target_transform_guess_dialog_->widget->save().as<Eigen::Isometry3d>();
        problem.intr = camera_intrinsics_dialog_->widget->save().as<CameraIntrinsics>();

        for(int i = 0; i < ui_->table_widget_data->rowCount(); ++i)
        {
            // Extract the table items
            QTableWidgetItem* features_item = ui_->table_widget_data->item(i, COL_FEATURES);
            QTableWidgetItem* homography_item = ui_->table_widget_data->item(i, COL_HOMOGRAPHY);
            QTableWidgetItem* notes_item = ui_->table_widget_data->item(i, COL_NOTES);

            QString image_file = features_item->data(IMAGE_FILE_NAME_ROLE).value<QString>();
            if(!QFile(image_file).exists())
            {
                notes_item->setData(Qt::EditRole, "Image file does not exist");
                continue;
            }

            QString pose_file = features_item->data(POSE_FILE_NAME_ROLE).value<QString>();
            if(!QFile(image_file).exists())
            {
                notes_item->setData(Qt::EditRole, "Pose file does not exist");
                continue;
            }

            try
            {
                // Load the image and pose
                cv::Mat image = cv::imread(image_file.toStdString());
                auto pose = YAML::LoadFile(pose_file.toStdString()).as<Eigen::Isometry3d>();

                // Populate an observation
                Observation2D3D obs;
                if(ui_->check_box_static_camera->isChecked())
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
                RandomCorrespondenceSampler random_sampler(obs.correspondence_set.size(),
                                                                                   obs.correspondence_set.size() / 3,
                                                                                   RANDOM_SEED);
                Eigen::VectorXd homography_error = calculateHomographyError(obs.correspondence_set, random_sampler);
                double homography_error_mean = homography_error.array().mean();

                // Conditionally add the observation to the problem if the mean homography error is less than the threshold
                if (homography_error_mean < ui_->double_spin_box_homography->value())
                    problem.observations.push_back(obs);

                // Update the table widget
                features_item->setData(Qt::EditRole, QString::number(obs.correspondence_set.size()));
                homography_item->setData(Qt::EditRole, QString::number(homography_error_mean));
                notes_item->setData(Qt::EditRole, "");
            }
            catch (const std::exception& ex)
            {
                notes_item->setData(Qt::EditRole, ex.what());
            }
        }

        // Solve the calibration problem
        result_ = std::make_shared<ExtrinsicHandEyeResult>();
        *result_ = optimize(problem);

        // Report results
        std::stringstream ss;
        ss << std::endl << *result_ << std::endl;
        ss << result_->covariance.printCorrelationCoeffAboveThreshold(0.5) << std::endl;

        // Compute the projected 3D error for comparison
        ss << analyze3dProjectionError(problem, *result_) << std::endl << std::endl;

        // Now let's compare the results of our extrinsic calibration with a PnP optimization for every observation.
        // The PnP optimization will give us an estimate of the camera to target transform using our input camera intrinsic
        // parameters We will then see how much this transform differs from the same transform calculated using the results
        // of the extrinsic calibration
        ExtrinsicHandEyeAnalysisStats stats = analyzeResults(problem, *result_);
        ss << stats << std::endl << std::endl;

        ui_->text_edit_log->clear();
        ui_->text_edit_log->append(QString::fromStdString(ss.str()));

        QApplication::restoreOverrideCursor();
        if (result_->converged)
            QMessageBox::information(this, "Success", "Successfully completed calibration");
        else
            QMessageBox::warning(this, "Error", "Calibration failed to converge");
    }
    catch(const std::exception& ex)
    {
        QApplication::restoreOverrideCursor();
        QMessageBox::warning(this, "Error", ex.what());
    }
}

void ICWidget::saveResults()
{
    if(result_ == nullptr)
    {
        QMessageBox::warning(this, "Error", "Calibration problem has not yet been solved. Please load the calibration data and run the calibration");
        return;
    }

    const QString file = QFileDialog::getSaveFileName(this, QString(), QString(), "YAML files (*.yaml *.yml)");
    std::ofstream f(file.toStdString());
    f << YAML::Node(*result_);
}

} // namespace industrial_calibration
