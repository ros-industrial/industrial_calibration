#ifndef ARUCO_TARGET_H
#define ARUCO_TARGET_H

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace Ui {
class ArucoTarget;
}

class ArucoTarget : public QWidget
{
  Q_OBJECT

public:
  explicit ArucoTarget(QWidget *parent = nullptr);
  ~ArucoTarget();

  void configure(const YAML::Node& node);
  YAML::Node save();

private:
  Ui::ArucoTarget *ui_;
};

#endif // ARUCO_TARGET_H
