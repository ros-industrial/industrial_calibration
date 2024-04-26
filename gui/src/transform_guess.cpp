#include "ui_transform_guess.h"
#include <industrial_calibration/gui/transform_guess.h>
#include <industrial_calibration/core/serialization.h>

namespace industrial_calibration
{
TransformGuess::TransformGuess(QWidget* parent) : ConfigurableWidget(parent), ui_(new Ui::TransformGuess)
{
  ui_->setupUi(this);
}

TransformGuess::~TransformGuess() { delete ui_; }

void TransformGuess::configure(const YAML::Node& node)
{
  ui_->double_spin_box_x->setValue(getMember<double>(node, "x"));
  ui_->double_spin_box_y->setValue(getMember<double>(node, "y"));
  ui_->double_spin_box_z->setValue(getMember<double>(node, "z"));
  ui_->double_spin_box_qx->setValue(getMember<double>(node, "qx"));
  ui_->double_spin_box_qy->setValue(getMember<double>(node, "qy"));
  ui_->double_spin_box_qz->setValue(getMember<double>(node, "qz"));
  ui_->double_spin_box_qw->setValue(getMember<double>(node, "qw"));
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

}  // namespace industrial_calibration
