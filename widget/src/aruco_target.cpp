#include "widget/aruco_target.h"
#include "ui_aruco_target.h"

ArucoTarget::ArucoTarget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ArucoTarget)
{
  ui->setupUi(this);
}

ArucoTarget::~ArucoTarget()
{
  delete ui;
}
