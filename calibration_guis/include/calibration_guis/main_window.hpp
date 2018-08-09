/**
 * @file /include/calibration_guis/main_window.hpp
 *
 * @brief Qt based gui for calibration_guis.
 *
 * @date November 2010
 **/
#ifndef calibration_guis_MAIN_WINDOW_H
#define calibration_guis_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace calibration_guis {


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:

  ros::ServiceClient start_client1_;
  ros::ServiceClient obs_client2_;
  ros::ServiceClient run_client3_;
  ros::ServiceClient save_client4_;
  ros::ServiceClient cov_client5_;

  ros::NodeHandle *nh_;
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
  //void on_button_connect_clicked(bool check );
  //void on_checkbox_use_environment_stateChanged(int state);
  //void on_rosTesting_clicked();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

   void on_startButton_clicked();
   void on_obsButton_clicked();
   void on_saveButton_clicked();
   void on_covButton_clicked();
   void on_runButton_clicked();
   void on_testingButton_clicked();

private:
    void on_pushButton_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace calibration_guis

#endif // calibration_guis_MAIN_WINDOW_H
