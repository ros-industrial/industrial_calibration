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
  ui_->double_spin_box_fx->setValue(node["fx"].as<double>());
  ui_->double_spin_box_fy->setValue(node["fy"].as<double>());
  ui_->double_spin_box_cx->setValue(node["cx"].as<double>());
  ui_->double_spin_box_cy->setValue(node["cy"].as<double>());
}

YAML::Node CameraIntrinsicsWidget::save() const
{
  YAML::Node node;
  node["fx"] = ui_->double_spin_box_fx->value();
  node["fy"] = ui_->double_spin_box_fy->value();
  node["cx"] = ui_->double_spin_box_cx->value();
  node["cy"] = ui_->double_spin_box_cy->value();

  return node;
}
