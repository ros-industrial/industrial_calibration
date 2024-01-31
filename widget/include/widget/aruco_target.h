#ifndef ARUCO_TARGET_H
#define ARUCO_TARGET_H

#include <QWidget>

namespace Ui {
class ArucoTarget;
}

class ArucoTarget : public QWidget
{
  Q_OBJECT

public:
  explicit ArucoTarget(QWidget *parent = nullptr);
  ~ArucoTarget();

private:
  Ui::ArucoTarget *ui;
};

#endif // ARUCO_TARGET_H
