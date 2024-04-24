#include <industrial_calibration/gui/charuco_grid_target_finder.h>
#include "ui_charuco_grid_target_finder.h"

namespace industrial_calibration
{
CharucoGridTargetFinderWidget::CharucoGridTargetFinderWidget(QWidget *parent) :
    ConfigurableWidget(parent),
    ui_(new Ui::CharucoGridTargetFinder)
{
    ui_->setupUi(this);
}

CharucoGridTargetFinderWidget::~CharucoGridTargetFinderWidget()
{
    delete ui_;
}

void CharucoGridTargetFinderWidget::configure(const YAML::Node& node)
{
    ui_->rowSpinBox->setValue(node["rows"].as<int>());
    ui_->colSpinBox->setValue(node["cols"].as<int>());
    ui_->checkerSizeDoubleSpinBox->setValue(node["chessboard_dim"].as<double>());
    ui_->markerSizeDoubleSpinBox->setValue(node["aruco_marker_dim"].as<double>());
    ui_->dictComboBox->setCurrentIndex(node["dictionary"].as<int>());
}

YAML::Node CharucoGridTargetFinderWidget::save() const
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

} // namespace industrial_calibration
