#include "widget/camera_intrinsics.h"
#include "ui_camera_intrinsics.h"

CameraIntrinsicsWidget::CameraIntrinsicsWidget(QWidget *parent) :
  ConfigurableWidget(parent),
  ui_(new Ui::CameraIntrinsics)
{
  ui_->setupUi(this);
}

CameraIntrinsicsWidget::~CameraIntrinsicsWidget()
{
  delete ui_;
}

void CameraIntrinsicsWidget::configure(const YAML::Node& node)
{
  ui_->fxDoubleSpinBox->setValue(node["fx"].as<double>());
  ui_->fyDoubleSpinBox->setValue(node["fy"].as<double>());
  ui_->cxDoubleSpinBox->setValue(node["cx"].as<double>());
  ui_->cyDoubleSpinBox->setValue(node["cy"].as<double>());
}

YAML::Node CameraIntrinsicsWidget::save() const
{
  YAML::Node node;
  node["fx"] = ui_->fxDoubleSpinBox->value();
  node["fy"] = ui_->fxDoubleSpinBox->value();
  node["cx"] = ui_->fxDoubleSpinBox->value();
  node["cy"] = ui_->fxDoubleSpinBox->value();

  return node;
}
