#pragma once

#include "configurable_widget.h"

namespace Ui {
class TransformGuess;
}

namespace industrial_calibration
{
class TransformGuess : public ConfigurableWidget
{
public:
    explicit TransformGuess(QWidget *parent = nullptr);
    ~TransformGuess();

    void configure(const YAML::Node& node) override;
    YAML::Node save() const override;

private:
    Ui::TransformGuess *ui_;
};

} // namespace industrial_calibration
