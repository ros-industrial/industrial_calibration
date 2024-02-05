#ifndef CAMERA_INTRINSICS_H
#define CAMERA_INTRINSICS_H

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace Ui {
class CameraIntrinsics;
}

class CameraIntrinsics : public QWidget
{
  Q_OBJECT

public:
  explicit CameraIntrinsics(QWidget *parent = nullptr);
  ~CameraIntrinsics();

    
  void configure(const YAML::Node& node);
  YAML::Node save();
  
private:
  Ui::CameraIntrinsics *ui_;
};

#endif // CAMERA_INTRINSICS_H
