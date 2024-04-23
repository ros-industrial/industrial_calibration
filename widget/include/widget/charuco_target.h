#ifndef CHARUCO_TARGET_H
#define CHARUCO_TARGET_H

#include "configurable_widget.h"

namespace Ui {
class CharucoTarget;
}

class CharucoTarget : public ConfigurableWidget
{
public:
  explicit CharucoTarget(QWidget *parent = nullptr);
  ~CharucoTarget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::CharucoTarget *ui_;
};

#endif // CHARUCO_TARGET_H
