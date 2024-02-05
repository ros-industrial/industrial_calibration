#include "widget/transform_guess.h"
#include "ui_transform_guess.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

TransformGuess::TransformGuess(QWidget *parent) :
  QWidget(parent),
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
  ui_->xDoubleSpinBox->setValue(node["x"].as<double>());
  ui_->yDoubleSpinBox->setValue(node["y"].as<double>());
  ui_->zDoubleSpinBox->setValue(node["z"].as<double>());

  // Convert from quaternion to rpy
  double qx = node["qx"].as<double>();
  double qy = node["qy"].as<double>();
  double qz = node["qz"].as<double>();
  double qw = node["qw"].as<double>();
  Eigen::Quaterniond quaternion(qw, qx, qy, qz);

  // Convert quaternion to roll, pitch, and yaw
  Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2); // XYZ order

  ui_->rollDoubleSpinBox->setValue(euler[0]);
  ui_->pitchDoubleSpinBox->setValue(euler[1]);
  ui_->yawDoubleSpinBox->setValue(euler[2]);
}

YAML::Node TransformGuess::save()
{
  YAML::Node node;
  node["x"] = ui_->xDoubleSpinBox->value();
  node["y"] = ui_->yDoubleSpinBox->value();
  node["z"] = ui_->zDoubleSpinBox->value();
  double roll = ui_->rollDoubleSpinBox->value();
  double pitch = ui_->pitchDoubleSpinBox->value();
  double yaw = ui_->yawDoubleSpinBox->value();

  // Convert rpy to quaternion
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;

  node["qx"] = quaternion.x();
  node["qy"] = quaternion.y();
  node["qz"] = quaternion.z();
  node["qw"] = quaternion.w();

  return node;
}
