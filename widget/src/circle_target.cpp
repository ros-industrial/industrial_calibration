#include "widget/circle_target.h"
#include "ui_circle_target.h"

CircleTarget::CircleTarget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CircleTarget)
{
  ui->setupUi(this);
}

CircleTarget::~CircleTarget()
{
  delete ui;
}
