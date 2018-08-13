/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/calibration_guis/main_window.hpp"
#include "std_srvs/Trigger.h"
#include "industrial_extrinsic_cal/cal_srv_solve.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace calibration_guis {

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
	, qnode(argc,argv)
{
  ros::init(argc,argv,"calibration_guis");
 nh_= new ros::NodeHandle;

 std::string startService = "ICalSrvStart";
 std::string observeService = "ICalSrvObs";
 std::string runService = "ICalSrvRun";
 std::string saveService = "ICalSrvSave";
 std::string covService = "ICalSrvCov";

  start_client1_ = nh_->serviceClient<std_srvs::Trigger>(startService);
  obs_client2_   = nh_->serviceClient<std_srvs::Trigger>(observeService);
  run_client3_   = nh_->serviceClient<industrial_extrinsic_cal::cal_srv_solve>(runService);
  save_client4_  = nh_->serviceClient<std_srvs::Trigger>(saveService);
  cov_client5_   = nh_->serviceClient<std_srvs::Trigger>(covService);

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
}

MainWindow::~MainWindow() {
  delete nh_;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
    close();
}
/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "calibration_guis");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    bool remember = settings.value("remember_settings", false).toBool();
    bool checked = settings.value("use_environment_variables", false).toBool();
    if ( checked ) {
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "calibration_guis");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());


}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

void calibration_guis::MainWindow::on_startButton_clicked()
{
  std_srvs::Trigger srv;
  if (start_client1_.call(srv))
  {
    ROS_INFO("Output %s",srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Trigger");
  }

}

void calibration_guis::MainWindow::on_obsButton_clicked()
{
  std_srvs::Trigger srv;
  if (obs_client2_.call(srv))
  {
    ROS_INFO("Output %s",srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Trigger");
  }
}

void calibration_guis::MainWindow::on_saveButton_clicked()
{
  std_srvs::Trigger srv;
  if (save_client4_.call(srv))
  {
    ROS_INFO("Output %s",srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Trigger");
  }
}

void calibration_guis::MainWindow::on_covButton_clicked()
{
  std_srvs::Trigger srv;
  if (cov_client5_.call(srv))
  {
    ROS_INFO("Output %s",srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service Trigger");
  }
}

void calibration_guis::MainWindow::on_runButton_clicked()
{
  industrial_extrinsic_cal::cal_srv_solve srv;
      float allowed_residual_num =0;
      std::string allowed_residual_string = ui.allowed_residual_->text().toStdString();
      std::istringstream iss(allowed_residual_string);
      iss >> allowed_residual_num;
      srv.request.allowable_cost_per_observation = allowed_residual_num;

      if (run_client3_.call(srv))
      {
        ROS_INFO("Output %s",srv.response.message.c_str());
        char temp_char[100];
        sprintf(temp_char, "%lf",srv.response.final_cost_per_observation);
        std::string result(temp_char);
        ui.final_resid_lb_->setText(result.c_str());
        if(srv.response.success)
        {
          ui.final_resid_lb_->setStyleSheet("QLabel { background-color : white; color : black; }");
        }
        else
        {
          ui.final_resid_lb_->setStyleSheet("QLabel { background-color : red; color : black; }");
        }
      }
      else
      {
        ROS_ERROR("Failed to call service Trigger");
      }
}
}  // namespace calibration_guis

void calibration_guis::MainWindow::on_testingButton_clicked()
{
    ROS_INFO("testing");
}

