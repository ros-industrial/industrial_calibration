/**
 * @file /include/stand_alone_gui/main_window.hpp
 *
 * @brief Qt based gui for stand_alone_gui
 *
 * @date November 2010
 **/
#ifndef STAND_ALONE_GUI_MAIN_WINDOW_H
#define STAND_ALONE_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace stand_alone_gui {


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:

  ros::ServiceClient start_client_;
  ros::ServiceClient obs_client_;
  ros::ServiceClient run_client_;
  ros::ServiceClient save_client_;
  ros::ServiceClient cov_client_;
  ros::ServiceClient load_client_;

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
  /******************************************
    ** Manual connections
  *******************************************/
  void updateLoggingView();
  void on_startButton_clicked();
  void on_obsButton_clicked();
  void on_saveButton_clicked();
  void on_covButton_clicked();
  void on_runButton_clicked();
  void on_loadButton_clicked();
private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace stand_alone_gui

#endif // stand_alone_gui_MAIN_WINDOW_H
