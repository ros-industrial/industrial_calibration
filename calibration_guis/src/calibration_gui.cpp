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


namespace calibration_guis
{
  calPanel::calPanel( QWidget* parent )
    : rviz::Panel( parent )
  {

    allowed_residual_       = new QLineEdit(parent);
    calibration_selection_  = new QComboBox(parent);

    // this client does not change with the type of calibration
    ros::NodeHandle pnh("~");
    ecp_client_   = pnh.serviceClient<std_srvs::Trigger>("store_mutable_joint_states");

    // these are the currently implemented types of service defined calibration routines
    // Note, WristCal, ICal and StereoCal expect a robot moves to change the scene
    // Either Camera or Target is on that robot's EOAT
    // In these cases TF provides IC for Camera to Target Transform
    // RailICal assumes a linear rail provides a sequence of observations equal distance apart with each call to OBS
    calibration_selection_->addItem("WristCalSrv"); // Either Camera or Target on EOAT, uses TF to get IC for Camera to Target Transform
    calibration_selection_->addItem("ICalSrv");     // Either Camera or Target on EOAT, uses TF to get IC for Camera to Target Transform
    calibration_selection_->addItem("StereoCalSrv");// Either Cameras or Target on EOAT, uses TF to get IC for Camera to Target Transform
    calibration_selection_->addItem("RailICalSrv"); // uses the 8D method where target to rail is changed after each sweep of rail, must happen at least once
    calibration_selection_->setCurrentIndex(0);
    reset_services();		// setup the services to use the default calibration type
    
    QPushButton* reset_button = new QPushButton( "Reset" , parent );
    QPushButton* observe_button = new QPushButton( "OBS" , parent );
    QPushButton* run_button = new QPushButton( "Run" , parent );
    QPushButton* save_button = new QPushButton( "Save" , parent );
    QPushButton* load_button = new QPushButton( "Load" , parent );
    QPushButton* covariance_button = new QPushButton( "Cov" , parent );
    QPushButton* install_excal_button = new QPushButton( "Install EX-cal" , parent );

    reset_button->setToolTip("Forget all past observations");
    observe_button->setToolTip("Take an observation");
    run_button->setToolTip("Run the Ceres-Solver Optimization");
    save_button->setToolTip("Push the resulting solution (only permanent for intrinsic calibration)");
    load_button->setToolTip("Load previous observations, enabling additional observations");
    covariance_button->setToolTip("Compute the covariance matrix, output will be on terminal");
    install_excal_button->setToolTip("Calls store_mutable_joint_states to install the extrinsic cal results");

    allowed_resid_lb_ = new QLabel( "Allowed Residal:" );
    final_resid_lb_   = new QLabel( "Final Residal = NOT_RUN " );
    obs_msg_lb_       = new QLabel( "Observation Message: ");
    cal_type_sel_lb_  = new QLabel( "Calibration Type:");

    QGridLayout* controls_layout = new QGridLayout();

    controls_layout->addWidget( cal_type_sel_lb_,        0, 0 );
    controls_layout->addWidget( calibration_selection_,  0, 1 );
    
    controls_layout->addWidget( reset_button,      1, 0 ); // reset
    controls_layout->addWidget( load_button,      1, 1 ); // Load

    controls_layout->addWidget( observe_button,      2, 0 ); // OBS
    controls_layout->addWidget( obs_msg_lb_       ,      2, 1 ); // Observation Message:
    
    controls_layout->addWidget( allowed_resid_lb_ ,      3, 0 ); // Allowed Residual:
    controls_layout->addWidget( allowed_residual_ ,      3, 1 ); // Allowed Residual typed in

    controls_layout->addWidget( run_button,      4, 0 ); // Run
    controls_layout->addWidget( final_resid_lb_   ,      4, 1 ); // Final Residual:
    
    controls_layout->addWidget( save_button,      5, 0 ); // Save
    controls_layout->addWidget( install_excal_button,      5, 1 ); // Install EX-cal

    controls_layout->addWidget( covariance_button,      6, 0 ); // Cov

    setLayout( controls_layout );

    connect( reset_button, SIGNAL( clicked( bool )), this, SLOT( resetClicked())); 
    connect( observe_button, SIGNAL( clicked( bool )), this, SLOT( observeClicked()));
    connect( run_button, SIGNAL( clicked( bool )), this, SLOT( runClicked())); 
    connect( save_button, SIGNAL( clicked( bool )), this, SLOT( saveClicked()));
    connect( load_button, SIGNAL( clicked( bool )), this, SLOT( loadClicked()));
    connect( covariance_button, SIGNAL( clicked( bool )), this, SLOT( covClicked())); 
    connect( install_excal_button, SIGNAL( clicked( bool )), this, SLOT( saveExCalClicked())); // make extrinsics permanent
    connect( calibration_selection_, SIGNAL( currentIndexChanged( int )), this, SLOT( reset_services()));
  }
void calPanel::resetClicked ()
  {
    std_srvs::Trigger srv;
    if (start_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call start Trigger");
    }
  }

  void calPanel::observeClicked ()
  {
    std_srvs::Trigger srv;
    if (obs_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
      obs_msg_lb_->setText(srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call observe Trigger");
    }
  }

  void calPanel::runClicked ()
  {
    industrial_extrinsic_cal::cal_srv_solve srv;
    float allowed_residual_num =0;
    std::string allowed_residual_string = allowed_residual_->text().toStdString().c_str();
    std::istringstream iss(allowed_residual_string);
    iss >> allowed_residual_num;
    srv.request.allowable_cost_per_observation = allowed_residual_num;

    if (run_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
      char temp_char[100];
      sprintf(temp_char, "Final Residual: %lf",srv.response.final_cost_per_observation);
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
      ROS_ERROR("Failed to call run Trigger");
    }
  }

  void calPanel::saveClicked ()
  {
    std_srvs::Trigger srv;
    if (save_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call save Trigger");
    }
  }

  void calPanel::loadClicked ()
  {
    std_srvs::Trigger srv;
    if (load_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call load Trigger");
    }
  }

  void calPanel::covClicked ()
  {
    std_srvs::Trigger srv;
    if (cov_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call covariance Trigger");
    }
  }
  void calPanel::saveExCalClicked ()
  {
    std_srvs::Trigger srv;
    if (ecp_client_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call store_mutable_joint_states");
    }
   }

  void calPanel::reset_services()
  {
    start_client_.shutdown();
    obs_client_.shutdown();  
    run_client_.shutdown();  
    save_client_.shutdown(); 
    load_client_.shutdown(); 
    cov_client_.shutdown();  

    ros::NodeHandle pnh("~");
    std::string bcn = calibration_selection_->currentText().toStdString().c_str();
    std::string start_service_name = "/" + bcn + "Start";
    std::string run_service_name   = "/" + bcn + "Run";
    std::string obs_service_name   = "/" + bcn + "Obs";
    std::string save_service_name  = "/" + bcn + "Save";
    std::string load_service_name  = "/" + bcn + "Load";
    std::string cov_service_name   = "/" + bcn + "Cov";

    start_client_ = pnh.serviceClient<std_srvs::Trigger>(start_service_name);
    obs_client_   = pnh.serviceClient<std_srvs::Trigger>(obs_service_name);
    run_client_   = pnh.serviceClient<industrial_extrinsic_cal::cal_srv_solve>(run_service_name);
    save_client_  = pnh.serviceClient<std_srvs::Trigger>(save_service_name);
    load_client_  = pnh.serviceClient<std_srvs::Trigger>(load_service_name);
    cov_client_   = pnh.serviceClient<std_srvs::Trigger>(cov_service_name);

    ROS_INFO("Now using %s services", bcn.c_str());

  }

  
} // end of calibration_guis namespace

PLUGINLIB_EXPORT_CLASS(calibration_guis::calPanel,rviz::Panel)
