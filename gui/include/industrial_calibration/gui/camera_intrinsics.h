#pragma once

#include <industrial_calibration/gui/configurable_widget.h>

namespace Ui {
class CameraIntrinsics;
}

namespace industrial_calibration
{
class CameraIntrinsicsWidget : public ConfigurableWidget
{
public:
    explicit CameraIntrinsicsWidget(QWidget *parent = nullptr);
    ~CameraIntrinsicsWidget();

    void configure(const YAML::Node& node) override;
    YAML::Node save() const override;

private:
    Ui::CameraIntrinsics *ui_;
};

} // namespace industrial_calibration
