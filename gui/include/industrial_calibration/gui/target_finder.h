#pragma once

#include <industrial_calibration/gui/configurable_widget.h>

#include <yaml-cpp/yaml.h>

namespace Ui
{
class TargetFinder;
}

namespace industrial_calibration
{
class TargetFinderWidget : public ConfigurableWidget
{
public:
  TargetFinderWidget(QWidget* parent = nullptr);
  virtual ~TargetFinderWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::TargetFinder* ui_;
};

}  // namespace industrial_calibration
