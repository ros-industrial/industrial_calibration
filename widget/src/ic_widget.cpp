#include "widget/ic_widget.h"
#include "ui_ic_widget.h"

#include <QDialog>
#include <QScrollBar>
#include "widget/transform_guess.h"
#include "widget/camera_intrinsics.h"
#include "widget/charuco_target.h"
#include <boost_plugin_loader/plugin_loader.h>

template<typename WidgetT>
class ICDialog : public QDialog
{
public:
  ICDialog(QWidget* parent = nullptr) : QDialog(parent)
  {
    auto* vl = new QVBoxLayout(this);
    auto* w = new WidgetT(this);
    vl->addWidget(w);
  }
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
  connect(ui_->loadPushButton, &QPushButton::clicked, this, &ICWidget::loadConfig);
  connect(ui_->saveConfigPushButton, &QPushButton::clicked, this, &ICWidget::saveConfig);
  connect(ui_->nextPushButton, &QPushButton::clicked, this, &ICWidget::getNextSample);
  connect(ui_->calibratePushButton, &QPushButton::clicked, this, &ICWidget::calibrate);
  connect(ui_->saveResultsPushButton, &QPushButton::clicked, this, &ICWidget::saveResults);

  // Ser up dialog boxes
  setup<TransformGuess>(this, camera_transform_guess_dialog_, ui_->cameraGuessPushButton);
  setup<TransformGuess>(this, target_transform_guess_dialog_, ui_->targetGuessToolButton);
  setup<CameraIntrinsics>(this, camera_intrinsics_dialog_, ui_->CameraIntrinsicsToolButton);
  setup<CharucoTarget>(this, charuco_target_dialog_);

  connect(ui_->targetToolButton, &QAbstractButton::clicked, [this](){
    QString target_type = ui_->targetComboBox->currentText();
    if(target_type == "ChArUco Grid")
      charuco_target_dialog_->show();
    else if(target_type == "ArUco Grid")
      emit log("hey you should create the ArUco widget");
    else
      emit log("hey you should create the circle widget");
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
  emit log("you pressed the load button");
}

void ICWidget::saveConfig()
{
  emit log("you pressed the save config button");
}

void ICWidget::getNextSample()
{
  emit log("you pressed the load button");
}

void ICWidget::calibrate()
{
  emit log("you pressed the calibrate button");
}

void ICWidget::saveResults()
{
  emit log("you pressed the save results button");
}
