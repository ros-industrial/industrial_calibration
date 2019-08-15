#ifndef CALIBRATION_GUI_H
#define CALIBRATION_GUI_H

#include <ros/ros.h>
#include <QLabel>
#include <rviz/panel.h>


class QLineEdit;

namespace calibration_guis
{
  class calPanel: public rviz::Panel
  {
    Q_OBJECT
    public:
       QLabel* obs_msg_lb_;
       QLabel* final_resid_lb_;
       calPanel( QWidget* parent = 0 );

       ros::ServiceClient start_client_;
       ros::ServiceClient run_client_;
       ros::ServiceClient obs_client_;
       ros::ServiceClient save_client_;
       ros::ServiceClient load_client_;
       ros::ServiceClient cov_client_;
       ros::ServiceClient ecp_client_;
    public Q_SLOTS:
      void setbutton1Clicked ();
      void setbutton2Clicked ();
      void setbutton3Clicked ();
      void setbutton4Clicked ();
      void setbutton5Clicked ();
      void setbutton6Clicked ();
      void setbutton7Clicked ();
    private:
      QLineEdit* allowed_residual_;


  };
} //end namespace calibration_guis

#endif
