#include "ui_aruco_grid_target_finder.h"
#include <industrial_calibration/gui/aruco_grid_target_finder.h>
#include <industrial_calibration/core/serialization.h>

namespace industrial_calibration
{
ArucoGridTargetFinderWidget::ArucoGridTargetFinderWidget(QWidget* parent)
  : ConfigurableWidget(parent), ui_(new Ui::ArucoGridTargetFinder)
{
  ui_->setupUi(this);
}

ArucoGridTargetFinderWidget::~ArucoGridTargetFinderWidget() { delete ui_; }

void ArucoGridTargetFinderWidget::configure(const YAML::Node& node)
{
  ui_->spin_box_rows->setValue(getMember<int>(node, "rows"));
  ui_->spin_box_cols->setValue(getMember<int>(node, "cols"));
  ui_->double_spin_box_marker_size->setValue(getMember<double>(node, "aruco_marker_dim"));
  ui_->double_spin_box_marker_gap->setValue(getMember<double>(node, "marker_gap"));
  ui_->combo_box_dict->setCurrentIndex(getMember<int>(node, "dictionary"));
}

YAML::Node ArucoGridTargetFinderWidget::save() const
{
  YAML::Node node;
  node["type"] = "ArucoGridTargetFinder";
  node["rows"] = ui_->spin_box_rows->value();
  node["cols"] = ui_->spin_box_cols->value();
  node["aruco_marker_dim"] = ui_->double_spin_box_marker_size->value();
  node["marker_gap"] = ui_->double_spin_box_marker_gap->value();
  node["dictionary"] = ui_->combo_box_dict->currentIndex();

  return node;
}

}  // namespace industrial_calibration
