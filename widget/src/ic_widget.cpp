#include "widget/ic_widget.h"
#include "ui_ic_widget.h"

#include <QDialog>
#include <QScrollBar>
#include <QStandardPaths>
#include <QFileDialog>
#include <fstream>
#include "widget/transform_guess.h"
#include "widget/camera_intrinsics.h"
#include "widget/charuco_target.h"
#include "widget/aruco_target.h"
#include "widget/circle_target.h"

template<typename WidgetT>
class ICDialog : public QDialog
{
public:
  ICDialog(QWidget* parent = nullptr) : QDialog(parent)
  {
    auto* vl = new QVBoxLayout(this);
    widget = new WidgetT(this);
    vl->addWidget(widget);
  }
  WidgetT* widget;
};

template<typename WidgetT>
void setup(QWidget* const parent_widget, QDialog*& dialog, QAbstractButton* const button = nullptr)
{
  dialog = new ICDialog<WidgetT>(parent_widget);
  dialog->setWindowTitle("");
  if (button != nullptr)  // check if button is not for target detector dialog
  {
    QObject::connect(button, &QAbstractButton::clicked, dialog, &QWidget::show);
  }
};

ICWidget::ICWidget(QWidget *parent) :
  QWidget(parent),
  ui_(new Ui::ICWidget)
{
  ui_->setupUi(this);

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->textEditLog->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->textEditLog->verticalScrollBar()->setSliderPosition(ui_->textEditLog->verticalScrollBar()->maximum());
  });

  // Set up push buttons
  connect(this, &ICWidget::log, this, &ICWidget::onUpdateLog);
  connect(ui_->loadConfigPushButton, &QPushButton::clicked, this, &ICWidget::loadConfig);
  connect(ui_->saveConfigPushButton, &QPushButton::clicked, this, &ICWidget::saveConfig);
  connect(ui_->nextPushButton, &QPushButton::clicked, this, &ICWidget::getNextSample);
  connect(ui_->calibratePushButton, &QPushButton::clicked, this, &ICWidget::calibrate);
  connect(ui_->saveResultsPushButton, &QPushButton::clicked, this, &ICWidget::saveResults);

  // Set up dialog boxes
  setup<TransformGuess>(this, camera_transform_guess_dialog_, ui_->cameraGuessPushButton);
  setup<TransformGuess>(this, target_transform_guess_dialog_, ui_->targetGuessToolButton);
  setup<CameraIntrinsics>(this, camera_intrinsics_dialog_, ui_->CameraIntrinsicsToolButton);
  setup<CharucoTarget>(this, charuco_target_dialog_);
  setup<ArucoTarget>(this, aruco_target_dialog_);
  setup<CircleTarget>(this, circle_target_dialog_);

  connect(ui_->targetToolButton, &QAbstractButton::clicked, [this](){
    QString type = ui_->targetComboBox->currentText();
    if(type == "CharucoGridTargetFinder")
      charuco_target_dialog_->show();
    else if(type == "ArucoGridTargetFinder")
      aruco_target_dialog_->show();
    else
      circle_target_dialog_->show();
  });

}

ICWidget::~ICWidget()
{
  delete ui_;
}

void ICWidget::onUpdateLog(const QString& message)
{
  ui_->textEditLog->append(message);
}

void ICWidget::loadConfig()
{
  // Get yaml filepath
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getOpenFileName(this, "Load calibration config", home, "YAML files (*.yaml *.yml)");
  if (file.isNull())
  {
    emit log("Unable to load file, filepath is null");
    return;
  }
  ui_->configLineEdit->setText(file);
  YAML::Node node = YAML::LoadFile(file.toStdString());
  
  // Load parameters
  {
    // Camera intrinsics
    auto dialog = dynamic_cast<ICDialog<CameraIntrinsics>*>(camera_intrinsics_dialog_);
    dialog->widget->configure(node["intrinsics"]);
  }

  {
    // Camera guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(camera_transform_guess_dialog_);
    dialog->widget->configure(node["camera_mount_to_camera_guess"]);
  }

  {
    // Target guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(target_transform_guess_dialog_);
    dialog->widget->configure(node["target_mount_to_target_guess"]);
  }

  // Homography
  ui_->homographyDoubleSpinBox->setValue(node["homography_threshold"].as<double>());

  {
    // Target
    std::string type = node["target_finder"]["type"].as<std::string>();
    // Target combo box: Charuco -> 0, Aruco -> 1, Circle -> 2
    if (type == "CharucoGridTargetFinder")
    {
      ui_->targetComboBox->setCurrentIndex(0);
      auto dialog = dynamic_cast<ICDialog<CharucoTarget>*>(charuco_target_dialog_);
      dialog->widget->configure(node["target_finder"]);
    }
    else if(type == "CharucoGridTargetFinder")
    {
      ui_->targetComboBox->setCurrentIndex(1);
      auto dialog = dynamic_cast<ICDialog<ArucoTarget>*>(aruco_target_dialog_);
      dialog->widget->configure(node["target_finder"]);
    }
    else if(type == "ModifiedCircleGridTargetFinder")
    { 
      ui_->targetComboBox->setCurrentIndex(2);
      auto dialog = dynamic_cast<ICDialog<CircleTarget>*>(circle_target_dialog_);
      dialog->widget->configure(node["target_finder"]);
    }
    else
    {
      emit log("Unrecognized target finder type"); // print target finder types as hint?
      return;
    }

  }
  emit log("calibration config loaded from: " + file);
}

void ICWidget::saveConfig()
{
  // Get filepath
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getSaveFileName(this, "Save calibration config", home, "YAML files (*.yaml *.yml)");
  
  if (file.isNull())
  {
    emit log("Invalid filepath, filepath is null");
    return;
  }
  
  YAML::Node node;
  {
    // Camera intrinsics
    auto dialog = dynamic_cast<ICDialog<CameraIntrinsics>*>(camera_intrinsics_dialog_);
    node["intrinsics"] = dialog->widget->save();
  }

  {
    // Camera guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(camera_transform_guess_dialog_);
    node["camera_mount_to_camera_guess"] = dialog->widget->save();
  }

  {
    // Target guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(target_transform_guess_dialog_);
    node["target_mount_to_target_guess"] = dialog->widget->save();
  }

  // Homography
  node["homography_threshold"] = ui_->homographyDoubleSpinBox->value();

  {
    // Target
    QString type = ui_->targetComboBox->currentText();
    if (type == "CharucoGridTargetFinder")
    {
      auto dialog = dynamic_cast<ICDialog<CharucoTarget>*>(charuco_target_dialog_);
      node["target_finder"] = dialog->widget->save();
    }
    else if(type == "ArucoGridTargetFinder")
    {
      auto dialog = dynamic_cast<ICDialog<ArucoTarget>*>(aruco_target_dialog_);
      node["target_finder"] = dialog->widget->save();
    }
    else
    { 
      auto dialog = dynamic_cast<ICDialog<CircleTarget>*>(circle_target_dialog_);
      node["target_finder"] = dialog->widget->save();
    }

  }
  std::ofstream fout(file.toStdString());
  fout << node;
  fout.close();
  
  emit log("Calibration config saved to: " + file);
}

void ICWidget::getNextSample()
{
  emit log("Save sample coming soon to a robot near you!");
}

void ICWidget::calibrate()
{
  emit log("Calibration coming soon to a robot near you!");
}

void ICWidget::saveResults()
{
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getSaveFileName(this, "Save results", home, "YAML files (*.yaml *.yml)");
  emit log("Save results coming soon to a robot near you!");
}
