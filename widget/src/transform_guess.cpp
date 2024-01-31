#include "widget/transform_guess.h"
#include "ui_transform_guess.h"

TransformGuess::TransformGuess(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::TransformGuess)
{
  ui->setupUi(this);
}

TransformGuess::~TransformGuess()
{
  delete ui;
}
