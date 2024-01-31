#include "widget/charuco_target.h"
#include "ui_charuco_target.h"

CharucoTarget::CharucoTarget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CharucoTarget)
{
  ui->setupUi(this);
}

CharucoTarget::~CharucoTarget()
{
  delete ui;
}
