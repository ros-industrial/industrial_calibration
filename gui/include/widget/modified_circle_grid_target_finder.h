#pragma once

#include "configurable_widget.h"

namespace Ui {
class ModifiedCircleGridTargetFinder;
}

namespace industrial_calibration
{
class ModifiedCircleGridTargetFinderWidget : public ConfigurableWidget
{
public:
    explicit ModifiedCircleGridTargetFinderWidget(QWidget *parent = nullptr);
    ~ModifiedCircleGridTargetFinderWidget();

    void configure(const YAML::Node& node) override;
    YAML::Node save() const override;

private:
    Ui::ModifiedCircleGridTargetFinder *ui_;
};

} // namespace industrial_calibration
