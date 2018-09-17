
#include <stdio.h>

#include <QColor>
#include <QSlider>
#include <QLabel>
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
#include <calibration_guis/drive_widget.h>
#include <calibration_guis/traj_panel.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(calibration_guis::TrajectoryPanel,rviz::Panel )

namespace calibration_guis
{


TrajectoryPanel::TrajectoryPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // Then create the control widget.
  drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( topic_layout );
    layout->addWidget( drive_widget_ );


    QPushButton* call_service_btn_1 = new QPushButton( "Capture" , parent );
    QPushButton* call_service_btn_2 = new QPushButton( "Execute" , parent );
    QPushButton* call_service_btn_3 = new QPushButton( "ExecuteWCall" , parent );
    QPushButton* call_service_btn_4 = new QPushButton( "MoveEnd" , parent );
    QPushButton* call_service_btn_5 = new QPushButton( "MoveNext" , parent );
    QPushButton* call_service_btn_6 = new QPushButton( "MovePrev" , parent );
    QPushButton* call_service_btn_7 = new QPushButton( "MoveStart" , parent );

    QGridLayout* controls_layout = new QGridLayout();
    controls_layout->addWidget( call_service_btn_1, 0, 0 );
    controls_layout->addWidget( call_service_btn_2, 1, 0 );
    controls_layout->addWidget( call_service_btn_3, 1, 1 );
    controls_layout->addWidget( call_service_btn_4, 3, 1 );
    controls_layout->addWidget( call_service_btn_5, 2, 0 );
    controls_layout->addWidget( call_service_btn_6, 2, 1 );
    controls_layout->addWidget( call_service_btn_7, 3, 0 );

    setLayout( controls_layout );


    QTimer* output_timer = new QTimer( this );

    // Next we make signal/slot connections.
    connect( call_service_btn_1, SIGNAL( clicked( bool )), this, SLOT( setbutton1Clicked()));
    connect( call_service_btn_2, SIGNAL( clicked( bool )), this, SLOT( setbutton2Clicked()));
    connect( call_service_btn_3, SIGNAL( clicked( bool )), this, SLOT( setbutton3Clicked()));
    connect( call_service_btn_4, SIGNAL( clicked( bool )), this, SLOT( setbutton4Clicked()));
    connect( call_service_btn_5, SIGNAL( clicked( bool )), this, SLOT( setbutton5Clicked()));
    connect( call_service_btn_6, SIGNAL( clicked( bool )), this, SLOT( setbutton6Clicked()));
    connect( call_service_btn_7, SIGNAL( clicked( bool )), this, SLOT( setbutton7Clicked()));

    connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
    connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
    connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

    // Start the timer.
    output_timer->start( 100 );

    // Make the control widget start disabled, since we don't start with an output topic.
    drive_widget_->setEnabled( false );

    ros::NodeHandle n("~");

    client1_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajCapture");
    client2_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajExecute");
    client3_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajExecuteWCall");
    client4_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMoveEnd");
    client5_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMoveNext");
    client6_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMovePrev");
    client7_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMoveStart");

  }

  // setVel() is connected to the DriveWidget's output, which is sent
  // whenever it changes due to a mouse event.  This just records the
  // values it is given.  The data doesn't actually get sent until the
  // next timer callback.
  void TrajectoryPanel::setVel( float lin, float ang )
  {
    linear_velocity_ = lin;
    angular_velocity_ = ang;
  }

  // Read the topic name from the QLineEdit and call setTopic() with the
  // results.  This is connected to QLineEdit::editingFinished() which
  // fires when the user presses Enter or Tab or otherwise moves focus
  // away.
  void TrajectoryPanel::updateTopic()
  {
    setTopic( output_topic_editor_->text() );
  }

  // Set the topic name we are publishing to.
  void TrajectoryPanel::setTopic( const QString& new_topic )
  {
    // Only take action if the name has changed.
    if( new_topic != output_topic_ )
    {
      output_topic_ = new_topic;
      // If the topic is the empty string, don't publish anything.
      if( output_topic_ == "" )
      {
        velocity_publisher_.shutdown();
      }
      else
      {
        // The old ``velocity_publisher_`` is destroyed by this assignment,
        // and thus the old topic advertisement is removed.  The call to
        // nh_advertise() says we want to publish data on the new topic
        // name.
        velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
      }
      // rviz::Panel defines the configChanged() signal.  Emitting it
      // tells RViz that something in this panel has changed that will
      // affect a saved config file.  Ultimately this signal can cause
      // QWidget::setWindowModified(true) to be called on the top-level
      // rviz::VisualizationFrame, which causes a little asterisk ("*")
      // to show in the window's title bar indicating unsaved changes.
      Q_EMIT configChanged();
    }

    // Gray out the control widget when the output topic is empty.
    drive_widget_->setEnabled( output_topic_ != "" );
  }

  // Publish the control velocities if ROS is not shutting down and the
  // publisher is ready with a valid topic name.
  void TrajectoryPanel::sendVel()
  {
    if( ros::ok() && velocity_publisher_ )
    {
      geometry_msgs::Twist msg;
      msg.linear.x = linear_velocity_;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = angular_velocity_;
      velocity_publisher_.publish( msg );
    }
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
  void TrajectoryPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    config.mapSetValue( "Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void TrajectoryPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    QString topic;
    if( config.mapGetString( "Topic", &topic ))
    {
      output_topic_editor_->setText( topic );
      updateTopic();
    }
  }
  void TrajectoryPanel::setbutton1Clicked ()
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

  void TrajectoryPanel::setbutton2Clicked ()
  {
    std_srvs::Trigger srv;
    if (client2_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void TrajectoryPanel::setbutton3Clicked ()
  {
    std_srvs::Trigger srv;
    if (client3_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void TrajectoryPanel::setbutton4Clicked ()
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

  void TrajectoryPanel::setbutton5Clicked ()
  {
    std_srvs::Trigger srv;
    if (client5_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void TrajectoryPanel::setbutton6Clicked ()
  {
    std_srvs::Trigger srv;
    if (client6_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }

  void TrajectoryPanel::setbutton7Clicked ()
  {
    std_srvs::Trigger srv;
    if (client7_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service Trigger");
    }
  }


} // end namespace calibration_guis



