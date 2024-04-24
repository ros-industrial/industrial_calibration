#pragma once

#include "configurable_widget.h"

namespace Ui {
class CharucoGridTargetFinder;
}

namespace industrial_calibration
{
class CharucoGridTargetFinderWidget : public ConfigurableWidget
{
public:
    explicit CharucoGridTargetFinderWidget(QWidget *parent = nullptr);
    ~CharucoGridTargetFinderWidget();

    void configure(const YAML::Node& node) override;
    YAML::Node save() const override;

private:
    Ui::CharucoGridTargetFinder *ui_;
};

} // namespace industrial_calibration
