#ifndef IC_WIDGET_H
#define IC_WIDGET_H

#include <industrial_calibration/target_finders/opencv/target_finder.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>
#include <QWidget>
#include <QDialog>

class QAbstractButton;
class AspectRatioPixmapLabel;

namespace Ui {
class ICWidget;
}

namespace industrial_calibration
{
class ExtrinsicHandEyeProblem2D3D;
class ExtrinsicHandEyeResult;
}

class ConfigurableWidget;

class ICDialog : public QDialog
{
public:
  ICDialog(ConfigurableWidget* widget_, QWidget* parent = nullptr);
  ConfigurableWidget* widget;
};

class ICWidget : public QWidget
{
public:
  explicit ICWidget(QWidget *parent = nullptr);
  ~ICWidget();

private:
  void loadConfig();
  void saveConfig();

  void loadData();
  void calibrate();

  void loadTargetFinder();
  void drawImage(int row, int col);
  void saveResults();

  Ui::ICWidget *ui_;
  AspectRatioPixmapLabel* image_label_;
  ICDialog* camera_transform_guess_dialog_;
  ICDialog* target_transform_guess_dialog_;
  ICDialog* camera_intrinsics_dialog_;
  std::map<QString, ICDialog*> target_dialogs_;

  boost_plugin_loader::PluginLoader loader_;
  industrial_calibration::TargetFinderFactoryOpenCV::ConstPtr factory_;

  std::shared_ptr<industrial_calibration::ExtrinsicHandEyeProblem2D3D> problem_;
  std::shared_ptr<industrial_calibration::ExtrinsicHandEyeResult> result_;
  industrial_calibration::TargetFinderOpenCV::ConstPtr target_finder_;
};

#endif // IC_WIDGET_H
