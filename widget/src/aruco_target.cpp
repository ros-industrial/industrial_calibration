#include "widget/aruco_target.h"
#include "ui_aruco_target.h"

ArucoTarget::ArucoTarget(QWidget *parent) :
  QWidget(parent),
  ui_(new Ui::ArucoTarget)
{
  ui_->setupUi(this);
}

ArucoTarget::~ArucoTarget()
{
  delete ui_;
}

void ArucoTarget::configure(const YAML::Node& node)
{
  ui_->rowSpinBox->setValue(node["rows"].as<int>());
  ui_->colSpinBox->setValue(node["cols"].as<int>());
  ui_->markerSizeDoubleSpinBox->setValue(node["aruco_marker_dim"].as<double>());
  ui_->dictComboBox->setCurrentIndex(node["dictionary"].as<int>());
}

YAML::Node ArucoTarget::save()
{
  YAML::Node node;
  node["type"] = "ArucoGridTargetFinder";
  node["rows"] = ui_->rowSpinBox->value();
  node["cols"] = ui_->colSpinBox->value();
  node["aruco_marker_dim"] = ui_->markerSizeDoubleSpinBox->value();
  node["dictionary"] = ui_->dictComboBox->currentIndex();

  return node;
}