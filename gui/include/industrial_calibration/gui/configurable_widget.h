#pragma once

#include <QDialog>
#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace industrial_calibration
{
class ConfigurableWidget : public QWidget
{
public:
  ConfigurableWidget(QWidget* parent = nullptr);

  virtual void configure(const YAML::Node& node) = 0;
  virtual YAML::Node save() const = 0;
};

class ConfigurableWidgetDialog : public QDialog
{
public:
  ConfigurableWidgetDialog(ConfigurableWidget* widget_, QWidget* parent = nullptr);
  ConfigurableWidget* widget;
};

}  // namespace industrial_calibration
