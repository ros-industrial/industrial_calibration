#ifndef CIRCLE_TARGET_H
#define CIRCLE_TARGET_H

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace Ui {
class CircleTarget;
}

class CircleTarget : public QWidget
{
  Q_OBJECT

public:
  explicit CircleTarget(QWidget *parent = nullptr);
  ~CircleTarget();

  void configure(const YAML::Node& node);
  YAML::Node save();

private:
  Ui::CircleTarget *ui_;
};

#endif // CIRCLE_TARGET_H
