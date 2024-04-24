#include "configurable_widget.h"

namespace Ui {
class CircleTarget;
}

namespace industrial_calibration
{
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

} // namespace industrial_calibration
