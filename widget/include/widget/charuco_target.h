#ifndef CHARUCO_TARGET_H
#define CHARUCO_TARGET_H

#include <QWidget>

namespace Ui {
class CharucoTarget;
}

class CharucoTarget : public QWidget
{
  Q_OBJECT

public:
  explicit CharucoTarget(QWidget *parent = nullptr);
  ~CharucoTarget();

private:
  Ui::CharucoTarget *ui;
};

#endif // CHARUCO_TARGET_H
