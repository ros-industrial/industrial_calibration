#include "ui_modified_circle_grid_target_finder.h"
#include <industrial_calibration/gui/modified_circle_grid_target_finder.h>
#include <industrial_calibration/core/serialization.h>

namespace industrial_calibration
{
ModifiedCircleGridTargetFinderWidget::ModifiedCircleGridTargetFinderWidget(QWidget *parent) :
    ConfigurableWidget(parent),
    ui_(new Ui::ModifiedCircleGridTargetFinder)
{
    ui_->setupUi(this);

    ui_->frame_filter_area->setEnabled(ui_->filterByAreaCheckBox->isChecked());
    ui_->frame_filter_circularity->setEnabled(ui_->filterByCircularityCheckBox->isChecked());
    ui_->frame_filter_color->setEnabled(ui_->filterByColorCheckBox->isChecked());
    ui_->frame_filter_convexity->setEnabled(ui_->filterByConvexityCheckBox->isChecked());
    ui_->frame_filter_inertia->setEnabled(ui_->filterByInertiaCheckBox->isChecked());

    connect(ui_->filterByAreaCheckBox, &QCheckBox::clicked, ui_->frame_filter_area, &QGroupBox::setEnabled);
    connect(ui_->filterByCircularityCheckBox, &QCheckBox::clicked, ui_->frame_filter_circularity, &QGroupBox::setEnabled);
    connect(ui_->filterByColorCheckBox, &QCheckBox::clicked, ui_->frame_filter_color, &QGroupBox::setEnabled);
    connect(ui_->filterByConvexityCheckBox, &QCheckBox::clicked, ui_->frame_filter_convexity, &QGroupBox::setEnabled);
    connect(ui_->filterByInertiaCheckBox, &QCheckBox::clicked, ui_->frame_filter_inertia, &QGroupBox::setEnabled);
}

ModifiedCircleGridTargetFinderWidget::~ModifiedCircleGridTargetFinderWidget()
{
    delete ui_;
}

void ModifiedCircleGridTargetFinderWidget::configure(const YAML::Node& node)
{
    ui_->rowSpinBox->setValue(node["rows"].as<int>());
    ui_->colSpinBox->setValue(node["cols"].as<int>());
    ui_->spacingDoubleSpinBox->setValue(node["spacing"].as<double>());

    auto& subnode = node["circle_detector_params"];

    ui_->minThresholdSpinBox->setValue(subnode["minThreshold"].as<double>());
    ui_->maxThresholdSpinBox->setValue(subnode["maxThreshold"].as<double>());
    ui_->numThresholdSpinBox->setValue(subnode["nThresholds"].as<int>());

    ui_->minRepeatSpinBox->setValue(subnode["minRepeatability"].as<int>());
    ui_->circleInclusionRadiusDoubleSpinBox->setValue(subnode["circleInclusionRadius"].as<double>());
    ui_->maxRadiusDiffDoubleSpinBox->setValue(subnode["maxRadiusDiff"].as<double>());
    ui_->maxAvgEllipseErrorDoubleSpinBox->setValue(subnode["maxAverageEllipseError"].as<double>());

    // Filter by color
    auto filter_by_color = getMember<bool>(subnode, "filterByColor");
    ui_->filterByColorCheckBox->setChecked(filter_by_color);
    ui_->frame_filter_color->setEnabled(filter_by_color);
    ui_->circleColorSpinBox->setValue(getMember<int>(subnode, "circleColor"));

    // Filter by area
    auto filter_by_area = getMember<bool>(subnode, "filterByArea");
    ui_->filterByAreaCheckBox->setChecked(filter_by_area);
    ui_->frame_filter_area->setEnabled(filter_by_area);
    ui_->minAreaDoubleSpinBox->setValue(getMember<double>(subnode, "minArea"));
    ui_->maxAreaDoubleSpinBox->setValue(getMember<double>(subnode, "maxArea"));

    // Filter by circularity
    auto filter_by_circularity = getMember<bool>(subnode, "filterByCircularity");
    ui_->filterByCircularityCheckBox->setChecked(filter_by_circularity);
    ui_->frame_filter_circularity->setEnabled(filter_by_circularity);
    ui_->minCircularityDoubleSpinBox->setValue(getMember<double>(subnode, "minCircularity"));
    ui_->maxCircularityDoubleSpinBox->setValue(getMember<double>(subnode, "maxCircularity"));

    // Filter by inertia
    auto filter_by_inertia = getMember<bool>(subnode, "filterByInertia");
    ui_->filterByInertiaCheckBox->setChecked(filter_by_inertia);
    ui_->frame_filter_inertia->setEnabled(filter_by_inertia);
    ui_->minInertiaRatioDoubleSpinBox->setValue(getMember<double>(subnode, "minInertiaRatio"));
    ui_->maxInertiaRatioDoubleSpinBox->setValue(getMember<double>(subnode, "maxInertiaRatio"));

    // Filter by convexity
    auto filter_by_convexity = getMember<bool>(subnode, "filterByConvexity");
    ui_->filterByConvexityCheckBox->setChecked(filter_by_convexity);
    ui_->frame_filter_convexity->setEnabled(filter_by_convexity);
    ui_->minConvexityDoubleSpinBox->setValue(getMember<double>(subnode, "minConvexity"));
    ui_->maxConvexityDoubleSpinBox->setValue(getMember<double>(subnode, "maxConvexity"));
}

YAML::Node ModifiedCircleGridTargetFinderWidget::save() const
{
    YAML::Node node;
    node["type"] = "ModifiedCircleGridTargetFinder";

    node["rows"] = ui_->rowSpinBox->value();
    node["cols"] = ui_->colSpinBox->value();
    node["spacing"] = ui_->spacingDoubleSpinBox->value();

    YAML::Node subnode;
    node["circle_detector_params"] = subnode;

    subnode["minThreshold"] = ui_->minThresholdSpinBox->value();
    subnode["maxThreshold"] = ui_->maxThresholdSpinBox->value();
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

} // namespace industrial_calibration
