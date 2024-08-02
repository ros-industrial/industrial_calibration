#include "ui_target_finder.h"
#include <industrial_calibration/core/serialization.h>
#include <industrial_calibration/gui/target_finder.h>
#include <industrial_calibration/gui/charuco_grid_target_finder.h>
#include <industrial_calibration/gui/aruco_grid_target_finder.h>
#include <industrial_calibration/gui/modified_circle_grid_target_finder.h>

namespace industrial_calibration
{
TargetFinderWidget::TargetFinderWidget(QWidget* parent) : ConfigurableWidget(parent), ui_(new Ui::TargetFinder())
{
  ui_->setupUi(this);

  ui_->stacked_widget->addWidget(new CharucoGridTargetFinderWidget(this));
  ui_->combo_box_target_finder->addItem("CharucoGridTargetFinder");

  ui_->stacked_widget->addWidget(new ArucoGridTargetFinderWidget(this));
  ui_->combo_box_target_finder->addItem("ArucoGridTargetFinder");

  ui_->stacked_widget->addWidget(new ModifiedCircleGridTargetFinderWidget(this));
  ui_->combo_box_target_finder->addItem("ModifiedCircleGridTargetFinder");

  connect(ui_->combo_box_target_finder, QOverload<int>::of(&QComboBox::currentIndexChanged), ui_->stacked_widget,
          QOverload<int>::of(&QStackedWidget::setCurrentIndex));
}

TargetFinderWidget::~TargetFinderWidget() { delete ui_; }

void TargetFinderWidget::configure(const YAML::Node& node)
{
  // Target
  QString target_type = QString::fromStdString(getMember<std::string>(node, "type"));
  int idx = ui_->combo_box_target_finder->findText(target_type);
  if (idx < 0)
    throw std::runtime_error("Unknown target type '" + target_type.toStdString() + "'");

  dynamic_cast<ConfigurableWidget*>(ui_->stacked_widget->widget(idx))->configure(node);
  ui_->combo_box_target_finder->setCurrentIndex(idx);
}

YAML::Node TargetFinderWidget::save() const
{
  QString target_type = ui_->combo_box_target_finder->currentText();
  return dynamic_cast<ConfigurableWidget*>(ui_->stacked_widget->currentWidget())->save();
}

}  // namespace industrial_calibration
