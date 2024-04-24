#pragma once

#include "configurable_widget.h"

namespace Ui {
class ArucoTarget;
}

namespace industrial_calibration
{
class ArucoTarget : public ConfigurableWidget
{
public:
    explicit ArucoTarget(QWidget *parent = nullptr);
    ~ArucoTarget();

    void configure(const YAML::Node& node) override;
    YAML::Node save() const override;

private:
    Ui::ArucoTarget *ui_;
};

} // namespace industrial_calibration
