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
class ExtrinsicHandEyeCalibrationConfigurationWidget;

class ExtrinsicHandEyeCalibrationWidget : public QWidget
{
public:
    explicit ExtrinsicHandEyeCalibrationWidget(QWidget *parent = nullptr);
    ~ExtrinsicHandEyeCalibrationWidget();

private:
    void loadConfig();

    void loadData();
    void calibrate();

    void loadTargetFinder();
    void drawImage(int row, int col);
    void saveResults();

    Ui::ExtrinsicHandEyeCalibration* ui_;
    ExtrinsicHandEyeCalibrationConfigurationWidget* configuration_widget_;

    boost_plugin_loader::PluginLoader loader_;
    TargetFinderFactoryOpenCV::ConstPtr factory_;

    std::shared_ptr<ExtrinsicHandEyeResult> result_;
    TargetFinderOpenCV::ConstPtr target_finder_;
};

} // namespace industrial_calibration
