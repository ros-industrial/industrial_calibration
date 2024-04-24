#include "ui_charuco_grid_target_finder.h"
#include <industrial_calibration/gui/charuco_grid_target_finder.h>
#include <industrial_calibration/core/serialization.h>

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
    ui_->rowSpinBox->setValue(getMember<int>(node, "rows"));
    ui_->colSpinBox->setValue(getMember<int>(node, "cols"));
    ui_->checkerSizeDoubleSpinBox->setValue(getMember<double>(node, "chessboard_dim"));
    ui_->markerSizeDoubleSpinBox->setValue(getMember<double>(node, "aruco_marker_dim"));
    ui_->dictComboBox->setCurrentIndex(getMember<int>(node, "dictionary"));
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
