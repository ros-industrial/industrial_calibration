#pragma once

#include "configurable_widget.h"

namespace Ui {
class ArucoGridTargetFinder;
}

namespace industrial_calibration
{
class ArucoGridTargetFinderWidget : public ConfigurableWidget
{
public:
    explicit ArucoGridTargetFinderWidget(QWidget *parent = nullptr);
    ~ArucoGridTargetFinderWidget();

    void configure(const YAML::Node& node) override;
    YAML::Node save() const override;

private:
    Ui::ArucoGridTargetFinder* ui_;
};

} // namespace industrial_calibration
