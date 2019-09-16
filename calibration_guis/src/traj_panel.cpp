
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
#include <calibration_guis/traj_panel.h>
#include <pluginlib/class_list_macros.h>



namespace calibration_guis
{


TrajectoryPanel::TrajectoryPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  output_topic_editor_           = new QLineEdit;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  topic_layout->addWidget( output_topic_editor_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );

  QPushButton* capture_btn           = new QPushButton( "Capture" , parent );
  QPushButton* execute_btn          = new QPushButton( "Execute" , parent );
  QPushButton* execute_wcall_btn = new QPushButton( "ExecuteWCall" , parent );
  QPushButton* move_end_btn       = new QPushButton( "MoveEnd" , parent );
  QPushButton* move_next_btn      = new QPushButton( "MoveNext" , parent );
  QPushButton* move_prev_btn      = new QPushButton( "MovePrev" , parent );
  QPushButton* move_start_btn      = new QPushButton( "MoveStart" , parent );
  
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( capture_btn,           0, 0 );
  controls_layout->addWidget( execute_btn,           1, 0 );
  controls_layout->addWidget( execute_wcall_btn,  1, 1 );
  controls_layout->addWidget( move_end_btn,        3, 1 );
  controls_layout->addWidget( move_next_btn,       2, 0 );
  controls_layout->addWidget( move_prev_btn,       2, 1 );
  controls_layout->addWidget( move_start_btn,       3, 0 );
  
  setLayout( controls_layout );
  
  
  // Next we make signal/slot connections.
  connect( capture_btn, SIGNAL( clicked( bool )), this, SLOT( captureClicked()));
  connect( execute_btn, SIGNAL( clicked( bool )), this, SLOT( executeClicked()));
  connect( execute_wcall_btn, SIGNAL( clicked( bool )), this, SLOT( executeWCallClicked()));
  connect( move_end_btn, SIGNAL( clicked( bool )), this, SLOT( moveEndClicked()));
  connect( move_next_btn, SIGNAL( clicked( bool )), this, SLOT( moveNextClicked()));
  connect( move_prev_btn, SIGNAL( clicked( bool )), this, SLOT( movePrevClicked()));
  connect( move_start_btn, SIGNAL( clicked( bool )), this, SLOT( moveStartClicked()));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
    
  ros::NodeHandle n("~");
  
  captureClient_      = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajCapture");
  executeClient_      = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajExecute");
  executeWCallClient_ = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajExecuteWCall");
  moveEndClient_      = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMoveEnd");
  moveNextClient_     = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMoveNext");
  movePrevClient_     = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMovePrev");
  moveStartClient_    = n.serviceClient<std_srvs::Trigger>("/joint_traj/JTrajMoveStart");
  
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
  void TrajectoryPanel::captureClicked ()
  {
    std_srvs::Trigger srv;
    if (captureClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call capture Trigger");
    }
  }

  void TrajectoryPanel::executeClicked ()
  {
    std_srvs::Trigger srv;
    if (executeClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call execute Trigger");
    }
  }

  void TrajectoryPanel::executeWCallClicked ()
  {
    std_srvs::Trigger srv;
    if (executeWCallClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call execute with call Trigger");
    }
  }

  void TrajectoryPanel::moveEndClicked ()
  {
    std_srvs::Trigger srv;
    if (moveEndClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call Move End Trigger");
    }
  }

  void TrajectoryPanel::moveNextClicked ()
  {
    std_srvs::Trigger srv;
    if (moveNextClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call Move Next Trigger");
    }
  }

  void TrajectoryPanel::movePrevClicked ()
  {
    std_srvs::Trigger srv;
    if (movePrevClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call Move Previous Trigger");
    }
  }

  void TrajectoryPanel::moveStartClicked ()
  {
    std_srvs::Trigger srv;
    if (moveStartClient_.call(srv))
    {
      ROS_INFO("Output %s",srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call Move Start Trigger");
    }
  }


} // end namespace calibration_guis


PLUGINLIB_EXPORT_CLASS(calibration_guis::TrajectoryPanel,rviz::Panel)
