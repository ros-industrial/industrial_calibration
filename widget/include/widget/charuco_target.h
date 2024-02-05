#ifndef CHARUCO_TARGET_H
#define CHARUCO_TARGET_H

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace Ui {
class CharucoTarget;
}

class CharucoTarget : public QWidget
{
  Q_OBJECT

public:
  explicit CharucoTarget(QWidget *parent = nullptr);
  ~CharucoTarget();

  void configure(const YAML::Node& node);
  YAML::Node save();

private:
  Ui::CharucoTarget *ui_;
};

#endif // CHARUCO_TARGET_H
