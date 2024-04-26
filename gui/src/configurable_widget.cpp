#include <industrial_calibration/gui/configurable_widget.h>
#include <QVBoxLayout>

namespace industrial_calibration
{
ConfigurableWidget::ConfigurableWidget(QWidget* parent) : QWidget(parent) {}

ConfigurableWidgetDialog::ConfigurableWidgetDialog(ConfigurableWidget* widget_, QWidget* parent)
  : QDialog(parent), widget(widget_)
{
  auto* vl = new QVBoxLayout(this);
  vl->addWidget(widget);
  setWindowTitle(" ");
}

}  // namespace industrial_calibration
