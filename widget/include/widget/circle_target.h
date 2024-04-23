#ifndef CIRCLE_TARGET_H
#define CIRCLE_TARGET_H

#include "configurable_widget.h"

namespace Ui {
class CircleTarget;
}

class CircleTarget : public ConfigurableWidget
{
public:
  explicit CircleTarget(QWidget *parent = nullptr);
  ~CircleTarget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::CircleTarget *ui_;
};

#endif // CIRCLE_TARGET_H
