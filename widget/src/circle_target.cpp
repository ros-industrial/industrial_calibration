#include "widget/circle_target.h"
#include "ui_circle_target.h"

CircleTarget::CircleTarget(QWidget *parent) :
  QWidget(parent),
  ui_(new Ui::CircleTarget)
{
  ui_->setupUi(this);
}

CircleTarget::~CircleTarget()
{
  delete ui_;
}

void CircleTarget::configure(const YAML::Node& node)
{
  ui_->rowSpinBox->setValue(node["rows"].as<int>());
  ui_->colSpinBox->setValue(node["cols"].as<int>());
  ui_->spacingDoubleSpinBox->setValue(node["spacing"].as<double>());

  ui_->minThresholdDoubleSpinBox->setValue(node["minThreshold"].as<double>());
  ui_->maxThresholdDoubleSpinBox->setValue(node["maxThreshold"].as<double>());
  ui_->numThresholdSpinBox->setValue(node["nThresholds"].as<int>());
  
  ui_->minRepeatSpinBox->setValue(node["minRepeatability"].as<int>());
  ui_->circleInclusionRadiusDoubleSpinBox->setValue(node["circleInclusionRadius"].as<double>());
  ui_->maxRadiusDiffDoubleSpinBox->setValue(node["maxRadiusDiff"].as<double>());

  ui_->maxAvgEllipseErrorDoubleSpinBox->setValue(node["maxAverageEllipseError"].as<double>());

  ui_->filterByColorCheckBox->setChecked(node["filterByColor"].as<bool>());  
  ui_->circleColorSpinBox->setValue(node["circleColor"].as<int>());

  ui_->filterByAreaCheckBox->setChecked(node["filterByArea"].as<bool>()); 
  ui_->minAreaDoubleSpinBox->setValue(node["minArea"].as<double>());
  ui_->maxAreaDoubleSpinBox->setValue(node["maxArea"].as<double>());

  ui_->filterByCircularityCheckBox->setChecked(node["filterByCircularity"].as<bool>()); 
  ui_->minCircularityDoubleSpinBox->setValue( node["minCircularity"].as<double>());
  ui_->maxCircularityDoubleSpinBox->setValue(node["maxCircularity"].as<double>());

  ui_->filterByInertiaCheckBox->setChecked(node["filterByInertia"].as<bool>());
  ui_->minInertiaRatioDoubleSpinBox->setValue(node["minInertiaRatio"].as<double>());
  ui_->maxInertiaRatioDoubleSpinBox->setValue(node["maxInertiaRatio"].as<double>());

  ui_->filterByCircularityCheckBox->setChecked(node["filterByConvexity"].as<bool>());
  ui_->minConvexityDoubleSpinBox->setValue(node["minConvexity"].as<double>());
  ui_->maxConvexityDoubleSpinBox->setValue(node["maxConvexity"].as<double>());
}

YAML::Node CircleTarget::save()
{
  YAML::Node node;
  node["type"] = "ModifiedCircleGridTargetFinder";
  
  node["rows"] = ui_->rowSpinBox->value();
  node["cols"] = ui_->colSpinBox->value();
  node["spacing"] = ui_->spacingDoubleSpinBox->value();

  node["minThreshold"] = ui_->minThresholdDoubleSpinBox->value();
  node["maxThreshold"] = ui_->maxThresholdDoubleSpinBox->value();
  node["nThresholds"] = ui_->numThresholdSpinBox->value();
  
  node["minRepeatability"] = ui_->minRepeatSpinBox->value();
  node["circleInclusionRadius"] = ui_->circleInclusionRadiusDoubleSpinBox->value();
  node["maxRadiusDiff"] = ui_->maxRadiusDiffDoubleSpinBox->value();

  node["maxAverageEllipseError"] = ui_->maxAvgEllipseErrorDoubleSpinBox->value();

  node["filterByColor"] = ui_->filterByColorCheckBox->isChecked();
  node["circleColor"] = ui_->circleColorSpinBox->value();

  node["filterByArea"] = ui_->filterByAreaCheckBox->isChecked();
  node["minArea"] = ui_->minAreaDoubleSpinBox->value();
  node["maxArea"] = ui_->maxAreaDoubleSpinBox->value();

  node["filterByCircularity"] = ui_->filterByCircularityCheckBox->isChecked();
  node["minCircularity"] = ui_->minCircularityDoubleSpinBox->value();
  node["maxCircularity"] = ui_->maxCircularityDoubleSpinBox->value();

  node["filterByInertia"] = ui_->filterByInertiaCheckBox->isChecked();
  node["minInertiaRatio"] = ui_->minInertiaRatioDoubleSpinBox->value();
  node["maxInertiaRatio"] = ui_->maxInertiaRatioDoubleSpinBox->value();

  node["filterByConvexity"] = ui_->filterByCircularityCheckBox->isChecked();
  node["minConvexity"] = ui_->minConvexityDoubleSpinBox->value();
  node["maxConvexity"] = ui_->maxConvexityDoubleSpinBox->value();

  return node;
}