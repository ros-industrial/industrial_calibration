#ifndef CALIBRATION_GUI_H
#define CALIBRATION_GUI_H

#include <ros/ros.h>
#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <rviz/panel.h>


class QLineEdit;

namespace calibration_guis
{
  class calPanel: public rviz::Panel
  {
    Q_OBJECT
    public:
       calPanel( QWidget* parent = 0 );

       // add the override functions TODO, perhaps these are not necessary
       void load(const rviz::Config& config)
       {
       };
       void save( rviz::Config config) const
       {
       };
       
       ros::ServiceClient start_client_;
       ros::ServiceClient run_client_;
       ros::ServiceClient obs_client_;
       ros::ServiceClient save_client_;
       ros::ServiceClient load_client_;
       ros::ServiceClient cov_client_;
       ros::ServiceClient ecp_client_;

    public Q_SLOTS:
      void resetClicked ();
      void observeClicked ();
      void runClicked ();
      void saveClicked ();
      void loadClicked ();
      void covClicked ();
      void saveExCalClicked ();
      void calibrationSelectionChanged();
      void reset_services();
      
    private:
      QLineEdit* allowed_residual_;
      QComboBox* calibration_selection_;
      QLabel* allowed_resid_lb_;
      QLabel* final_resid_lb_;
      QLabel* obs_msg_lb_;
      QLabel* cal_type_sel_lb_;

  };
} //end namespace calibration_guis

#endif
