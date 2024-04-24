#include "ui_aruco_grid_target_finder.h"
#include <industrial_calibration/gui/aruco_grid_target_finder.h>
#include <industrial_calibration/core/serialization.h>

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
    ui_->rowSpinBox->setValue(getMember<int>(node, "rows"));
    ui_->colSpinBox->setValue(getMember<int>(node, "cols"));
    ui_->markerSizeDoubleSpinBox->setValue(getMember<double>(node, "aruco_marker_dim"));
    ui_->dictComboBox->setCurrentIndex(getMember<int>(node, "dictionary"));
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

