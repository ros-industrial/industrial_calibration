#ifndef TRANSFORM_GUESS_H
#define TRANSFORM_GUESS_H

#include <QWidget>

namespace Ui {
class TransformGuess;
}

class TransformGuess : public QWidget
{
  Q_OBJECT

public:
  explicit TransformGuess(QWidget *parent = nullptr);
  ~TransformGuess();

private:
  Ui::TransformGuess *ui;
};

#endif // TRANSFORM_GUESS_H
