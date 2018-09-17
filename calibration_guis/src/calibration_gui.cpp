#include <stdio.h>

#include <QColor>
#include <QSlider>
#include <QGridLayout>
#include <QPushButton>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <industrial_extrinsic_cal/cal_srv_solve.h>

#include <calibration_guis/calibration_gui.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(calibration_guis::calPanel,rviz::Panel)

namespace calibration_guis
{
  calPanel::calPanel( QWidget* parent )
    : rviz::Panel( parent )
  {
    ros::NodeHandle pnh("~");
    std::string bcn;
    if(!pnh.getParam("base_calibration_name",bcn))
    {
      ROS_ERROR("MUST SET base_calibration_name");
      bcn="/IntrinsicCalSrv";
      pnh.setParam("base_calibration_name",bcn);
    }
    ROS_INFO("base_calibration_name: %s",bcn.c_str());

    std::string start_service_name = bcn + "Start";
    std::string run_service_name   = bcn + "Run";
    std::string obs_service_name   = bcn + "Obs";
    std::string save_service_name  = bcn + "Save";

    client1_ = pnh.serviceClient<std_srvs::Trigger>(start_service_name);
    client2_ = pnh.serviceClient<std_srvs::Trigger>(obs_service_name);
    ROS_INFO("%s", obs_service_name.c_str());
    client3_ = pnh.serviceClient<industrial_extrinsic_cal::cal_srv_solve>(run_service_name);
    client4_ = pnh.serviceClient<std_srvs::Trigger>(save_service_name);

    allowed_residual_ = new QLineEdit(parent);


    QPushButton* call_service_btn_1 = new QPushButton( "Start" , parent );
    QPushButton* call_service_btn_2 = new QPushButton( "OBS" , parent );
    QPushButton* call_service_btn_3 = new QPushButton( "Run" , parent );
    QPushButton* call_service_btn_4 = new QPushButton( "Save" , parent );
    QPushButton* call_service_btn_5 = new QPushButton( "Load" , parent );
    QPushButton* call_service_btn_6 = new QPushButton( "Cov" , parent );

    QLabel* allowed_resid_lb = new QLabel( "Allowed Residal" );
    final_resid_lb_ = new QLabel( "Final Residal: " );
    obs_msg_lb_ = new QLabel("Observation Message: ");
    QLabel* bcn_name_lb = new QLabel(bcn.c_str());

    QGridLayout* controls_layout = new QGridLayout();

    controls_layout->addWidget( call_service_btn_1, 1, 0 );
    controls_layout->addWidget( bcn_name_lb       , 0, 1);
    controls_layout->addWidget( call_service_btn_2, 2, 0 );
    controls_layout->addWidget( obs_msg_lb_       , 2, 1 );
    controls_layout->addWidget( call_service_btn_3, 3, 0 );
    controls_layout->addWidget( allowed_resid_lb  , 3, 1 );
    controls_layout->addWidget( allowed_residual_ , 3, 2 );
    controls_layout->addWidget( final_resid_lb_   , 4, 2 );
    controls_layout->addWidget( call_service_btn_4, 5, 0 );

    setLayout( controls_layout );

    connect( call_service_btn_1, SIGNAL( clicked( bool )), this, SLOT( setbutton1Clicked()));
    connect( call_service_btn_2, SIGNAL( clicked( bool )), this, SLOT( setbutton2Clicked()));
    connect( call_service_btn_3, SIGNAL( clicked( bool )), this, SLOT( setbutton3Clicked()));
    connect( call_service_btn_4, SIGNAL( clicked( bool )), this, SLOT( setbutton4Clicked()));

  }

  void calPanel::setbutton1Clicked ()
  {
    std_srvs::Trigger srv;
    if (client1_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void calPanel::setbutton2Clicked ()
  {
    std_srvs::Trigger srv;
    if (client2_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
      obs_msg_lb_->setText(srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void calPanel::setbutton3Clicked ()
  {
    industrial_extrinsic_cal::cal_srv_solve srv;
    float allowed_residual_num =0;
    std::string allowed_residual_string = allowed_residual_->text().toStdString().c_str();
    std::istringstream iss(allowed_residual_string);
    iss >> allowed_residual_num;
    srv.request.allowable_cost_per_observation = allowed_residual_num;

    if (client3_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
      char temp_char[100];
      sprintf(temp_char, "%lf",srv.response.final_cost_per_observation);
      std::string result(temp_char);

      final_resid_lb_->setText(result.c_str());
      if(srv.response.success)
      {
        final_resid_lb_->setStyleSheet("QLabel { background-color : white; color : black; }");
      }
      else
      {
        final_resid_lb_->setStyleSheet("QLabel { background-color : red; color : black; }");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void calPanel::setbutton4Clicked ()
  {
    std_srvs::Trigger srv;
    if (client4_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }
} // end of calibration_guis namespace
