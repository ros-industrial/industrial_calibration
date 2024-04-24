#include "configurable_widget.h"

namespace Ui {
class CharucoTarget;
}

namespace industrial_calibration
{
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

} // namespace industrial_calibration
