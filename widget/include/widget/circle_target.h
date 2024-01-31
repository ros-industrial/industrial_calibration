#ifndef CIRCLE_TARGET_H
#define CIRCLE_TARGET_H

#include <QWidget>

namespace Ui {
class CircleTarget;
}

class CircleTarget : public QWidget
{
  Q_OBJECT

public:
  explicit CircleTarget(QWidget *parent = nullptr);
  ~CircleTarget();

private:
  Ui::CircleTarget *ui;
};

#endif // CIRCLE_TARGET_H
