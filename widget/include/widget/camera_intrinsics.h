#ifndef CAMERA_INTRINSICS_H
#define CAMERA_INTRINSICS_H

#include "configurable_widget.h"

namespace Ui {
class CameraIntrinsics;
}

class CameraIntrinsicsWidget : public ConfigurableWidget
{
  Q_OBJECT

public:
  explicit CameraIntrinsicsWidget(QWidget *parent = nullptr);
  ~CameraIntrinsicsWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::CameraIntrinsics *ui_;
};

#endif // CAMERA_INTRINSICS_H
