#ifndef CAMERA_INTRINSICS_H
#define CAMERA_INTRINSICS_H

#include <QWidget>

namespace Ui {
class CameraIntrinsics;
}

class CameraIntrinsics : public QWidget
{
  Q_OBJECT

public:
  explicit CameraIntrinsics(QWidget *parent = nullptr);
  ~CameraIntrinsics();

private:
  Ui::CameraIntrinsics *ui;
};

#endif // CAMERA_INTRINSICS_H
