#include "widget/camera_intrinsics.h"
#include "ui_camera_intrinsics.h"

CameraIntrinsics::CameraIntrinsics(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CameraIntrinsics)
{
  ui->setupUi(this);
}

CameraIntrinsics::~CameraIntrinsics()
{
  delete ui;
}
