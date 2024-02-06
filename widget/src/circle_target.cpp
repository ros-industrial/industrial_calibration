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

  auto& subnode = node["circle_detector_params"];

  ui_->minThresholdDoubleSpinBox->setValue(subnode["minThreshold"].as<double>());
  ui_->maxThresholdDoubleSpinBox->setValue(subnode["maxThreshold"].as<double>());
  ui_->numThresholdSpinBox->setValue(subnode["nThresholds"].as<int>());
  
  ui_->minRepeatSpinBox->setValue(subnode["minRepeatability"].as<int>());
  ui_->circleInclusionRadiusDoubleSpinBox->setValue(subnode["circleInclusionRadius"].as<double>());
  ui_->maxRadiusDiffDoubleSpinBox->setValue(subnode["maxRadiusDiff"].as<double>());

  ui_->maxAvgEllipseErrorDoubleSpinBox->setValue(subnode["maxAverageEllipseError"].as<double>());

  ui_->filterByColorCheckBox->setChecked(subnode["filterByColor"].as<bool>());  
  ui_->circleColorSpinBox->setValue(subnode["circleColor"].as<int>());

  ui_->filterByAreaCheckBox->setChecked(subnode["filterByArea"].as<bool>()); 
  ui_->minAreaDoubleSpinBox->setValue(subnode["minArea"].as<double>());
  ui_->maxAreaDoubleSpinBox->setValue(subnode["maxArea"].as<double>());

  ui_->filterByCircularityCheckBox->setChecked(subnode["filterByCircularity"].as<bool>()); 
  ui_->minCircularityDoubleSpinBox->setValue( subnode["minCircularity"].as<double>());
  ui_->maxCircularityDoubleSpinBox->setValue(subnode["maxCircularity"].as<double>());

  ui_->filterByInertiaCheckBox->setChecked(subnode["filterByInertia"].as<bool>());
  ui_->minInertiaRatioDoubleSpinBox->setValue(subnode["minInertiaRatio"].as<double>());
  ui_->maxInertiaRatioDoubleSpinBox->setValue(subnode["maxInertiaRatio"].as<double>());

  ui_->filterByCircularityCheckBox->setChecked(subnode["filterByConvexity"].as<bool>());
  ui_->minConvexityDoubleSpinBox->setValue(subnode["minConvexity"].as<double>());
  ui_->maxConvexityDoubleSpinBox->setValue(subnode["maxConvexity"].as<double>());
}

YAML::Node CircleTarget::save()
{
  YAML::Node node;
  node["type"] = "ModifiedCircleGridTargetFinder";
  
  node["rows"] = ui_->rowSpinBox->value();
  node["cols"] = ui_->colSpinBox->value();
  node["spacing"] = ui_->spacingDoubleSpinBox->value();

  YAML::Node subnode;
  node["circle_detector_params"] = subnode;

  subnode["minThreshold"] = ui_->minThresholdDoubleSpinBox->value();
  subnode["maxThreshold"] = ui_->maxThresholdDoubleSpinBox->value();
  subnode["nThresholds"] = ui_->numThresholdSpinBox->value();
  
  subnode["minRepeatability"] = ui_->minRepeatSpinBox->value();
  subnode["circleInclusionRadius"] = ui_->circleInclusionRadiusDoubleSpinBox->value();
  subnode["maxRadiusDiff"] = ui_->maxRadiusDiffDoubleSpinBox->value();

  subnode["maxAverageEllipseError"] = ui_->maxAvgEllipseErrorDoubleSpinBox->value();

  subnode["filterByColor"] = ui_->filterByColorCheckBox->isChecked();
  subnode["circleColor"] = ui_->circleColorSpinBox->value();

  subnode["filterByArea"] = ui_->filterByAreaCheckBox->isChecked();
  subnode["minArea"] = ui_->minAreaDoubleSpinBox->value();
  subnode["maxArea"] = ui_->maxAreaDoubleSpinBox->value();

  subnode["filterByCircularity"] = ui_->filterByCircularityCheckBox->isChecked();
  subnode["minCircularity"] = ui_->minCircularityDoubleSpinBox->value();
  subnode["maxCircularity"] = ui_->maxCircularityDoubleSpinBox->value();

  subnode["filterByInertia"] = ui_->filterByInertiaCheckBox->isChecked();
  subnode["minInertiaRatio"] = ui_->minInertiaRatioDoubleSpinBox->value();
  subnode["maxInertiaRatio"] = ui_->maxInertiaRatioDoubleSpinBox->value();

  subnode["filterByConvexity"] = ui_->filterByCircularityCheckBox->isChecked();
  subnode["minConvexity"] = ui_->minConvexityDoubleSpinBox->value();
  subnode["maxConvexity"] = ui_->maxConvexityDoubleSpinBox->value();

  return node;
}