#include "widget/charuco_target.h"
#include "ui_charuco_target.h"

CharucoTarget::CharucoTarget(QWidget *parent) :
  QWidget(parent),
  ui_(new Ui::CharucoTarget)
{
  ui_->setupUi(this);
}

CharucoTarget::~CharucoTarget()
{
  delete ui_;
}

void CharucoTarget::configure(const YAML::Node& node)
{
  ui_->rowSpinBox->setValue(node["rows"].as<int>());
  ui_->colSpinBox->setValue(node["cols"].as<int>());
  ui_->checkerSizeDoubleSpinBox->setValue(node["chessboard_dim"].as<double>());
  ui_->markerSizeDoubleSpinBox->setValue(node["aruco_marker_dim"].as<double>());
  ui_->dictComboBox->setCurrentIndex(node["dictionary"].as<int>());
}

YAML::Node CharucoTarget::save()
{
  YAML::Node node;
  node["type"] = "CharucoGridTargetFinder";
  node["rows"] = ui_->rowSpinBox->value();
  node["cols"] = ui_->colSpinBox->value();
  node["chessboard_dim"] = ui_->checkerSizeDoubleSpinBox->value();
  node["aruco_marker_dim"] = ui_->markerSizeDoubleSpinBox->value();
  node["dictionary"] = ui_->dictComboBox->currentIndex();

  return node;
}
