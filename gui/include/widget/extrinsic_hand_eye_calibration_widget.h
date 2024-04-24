#pragma once

#include <industrial_calibration/target_finders/opencv/target_finder.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>
#include <QWidget>
#include <QDialog>

class QAbstractButton;

namespace Ui {
class ExtrinsicHandEyeCalibration;
}

namespace industrial_calibration
{
class ExtrinsicHandEyeResult;
class ConfigurableWidgetDialog;

class ExtrinsicHandEyeCalibrationWidget : public QWidget
{
public:
    explicit ExtrinsicHandEyeCalibrationWidget(QWidget *parent = nullptr);
    ~ExtrinsicHandEyeCalibrationWidget();

private:
    void loadConfig();
    void saveConfig();

    void loadData();
    void calibrate();

    void loadTargetFinder();
    void drawImage(int row, int col);
    void saveResults();

    Ui::ExtrinsicHandEyeCalibration* ui_;
    ConfigurableWidgetDialog* camera_transform_guess_dialog_;
    ConfigurableWidgetDialog* target_transform_guess_dialog_;
    ConfigurableWidgetDialog* camera_intrinsics_dialog_;
    std::map<QString, ConfigurableWidgetDialog*> target_dialogs_;

    boost_plugin_loader::PluginLoader loader_;
    TargetFinderFactoryOpenCV::ConstPtr factory_;

    std::shared_ptr<ExtrinsicHandEyeResult> result_;
    TargetFinderOpenCV::ConstPtr target_finder_;
};

} // namespace industrial_calibration
