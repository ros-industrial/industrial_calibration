#include "widget/transform_guess.h"
#include "ui_transform_guess.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

TransformGuess::TransformGuess(QWidget *parent) :
  ConfigurableWidget(parent),
  ui_(new Ui::TransformGuess)
{
  ui_->setupUi(this);
}

TransformGuess::~TransformGuess()
{
  delete ui_;
}

void TransformGuess::configure(const YAML::Node& node)
{
  ui_->double_spin_box_x->setValue(node["x"].as<double>());
  ui_->double_spin_box_y->setValue(node["y"].as<double>());
  ui_->double_spin_box_z->setValue(node["z"].as<double>());

  // Convert from quaternion to rpy
  ui_->double_spin_box_qx->setValue(node["qx"].as<double>());
  ui_->double_spin_box_qy->setValue(node["qy"].as<double>());
  ui_->double_spin_box_qz->setValue(node["qz"].as<double>());
  ui_->double_spin_box_qw->setValue(node["qw"].as<double>());
}

YAML::Node TransformGuess::save() const
{
  YAML::Node node;
  node["x"] = ui_->double_spin_box_x->value();
  node["y"] = ui_->double_spin_box_y->value();
  node["z"] = ui_->double_spin_box_z->value();
  node["qx"] = ui_->double_spin_box_qx->value();
  node["qy"] = ui_->double_spin_box_qy->value();
  node["qz"] = ui_->double_spin_box_qz->value();
  node["qw"] = ui_->double_spin_box_qw->value();

  return node;
}
