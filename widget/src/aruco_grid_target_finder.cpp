#include "widget/aruco_grid_target_finder.h"
#include "ui_aruco_grid_target_finder.h"

namespace industrial_calibration
{
ArucoGridTargetFinderWidget::ArucoGridTargetFinderWidget(QWidget *parent) :
    ConfigurableWidget(parent),
    ui_(new Ui::ArucoGridTargetFinder)
{
    ui_->setupUi(this);
}

ArucoGridTargetFinderWidget::~ArucoGridTargetFinderWidget()
{
    delete ui_;
}

void ArucoGridTargetFinderWidget::configure(const YAML::Node& node)
{
    ui_->rowSpinBox->setValue(node["rows"].as<int>());
    ui_->colSpinBox->setValue(node["cols"].as<int>());
    ui_->markerSizeDoubleSpinBox->setValue(node["aruco_marker_dim"].as<double>());
    ui_->dictComboBox->setCurrentIndex(node["dictionary"].as<int>());
}

YAML::Node ArucoGridTargetFinderWidget::save() const
{
    YAML::Node node;
    node["type"] = "ArucoGridTargetFinder";
    node["rows"] = ui_->rowSpinBox->value();
    node["cols"] = ui_->colSpinBox->value();
    node["aruco_marker_dim"] = ui_->markerSizeDoubleSpinBox->value();
    node["dictionary"] = ui_->dictComboBox->currentIndex();

    return node;
}

} // namespace industrial_calibration

