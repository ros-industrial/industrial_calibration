#ifndef IC_WIDGET_H
#define IC_WIDGET_H

#include <QWidget>

class QDialog;
class QAbstractButton;

namespace Ui {
class ICWidget;
}

class ICWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ICWidget(QWidget *parent = nullptr);
  ~ICWidget();

private:
  Ui::ICWidget *ui_;

  void loadConfig();
  void saveConfig();
  void calibrate();
  void updateProgressBar();
  void drawImage(const QString& filepath);
  void getNextSample();
  void saveResults();
  void updateLog(const QString& message);

  QDialog* camera_transform_guess_dialog_;
  QDialog* target_transform_guess_dialog_;
  QDialog* camera_intrinsics_dialog_;
  QDialog* charuco_target_dialog_;
  QDialog* aruco_target_dialog_;
  QDialog* circle_target_dialog_;
  QString data_dir;

};

#endif // IC_WIDGET_H
