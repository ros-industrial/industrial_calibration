#include <ros/ros.h>

#include <string>
#include <fstream>

// ros includes
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Trigger.h>

// calibration library includes
#include <industrial_extrinsic_cal/robot_joint_values_triggerAction.h>
#include <industrial_extrinsic_cal/yaml_utils.h>

#include <yaml-cpp/yaml.h>


typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::robot_joint_values_triggerAction>
    RobotJointValuesClient;

using industrial_extrinsic_cal::yamlNodeFromFileName;
using industrial_extrinsic_cal::parseNode;
using industrial_extrinsic_cal::parseVectorD;
using industrial_extrinsic_cal::parseString;

class JointTraj
{
public:
  // constructor
  JointTraj(ros::NodeHandle nh):
    nh_(nh), current_index_(-1)
  {
    nh_.param<std::string>("input_file", in_file_name_, "joint_traj.yaml");
    nh_.param<std::string>("output_file", out_file_name_, "joint_traj2.yaml");
    nh_.param<std::string>("motion_server", motion_server_name_, "/rosRobotSceneTrigger_joint_values");
    nh_.param<std::string>("trigger_server", trigger_server_name_, "/ICalSrvObs");
    nh_.param<std::string>("joints_topic", joints_topic_name_, "/joint_states");
    nh_.param<double>("pause_time", pause_time_, 0.0);
    joint_trajectory_data.clear();

    ofile_.open(out_file_name_.c_str());
    if(!ofile_.is_open()){
      ROS_ERROR("can't open %s", out_file_name_.c_str());
    }
    else{
      ROS_INFO("output file: %s", out_file_name_.c_str());
    }
    write_header();

    YAML::Node traj_doc;
    if (!yamlNodeFromFileName(in_file_name_, traj_doc))
      {
	ROS_ERROR("Can't parse yaml file %s", in_file_name_.c_str());
      }
    else
      {
	const YAML::Node& node = parseNode(traj_doc, "poses");
	if(node)
	  {
	    std::vector<double> joint_values;
	    for(unsigned int i=0; i < node.size(); i++){
	      joint_values.clear();
	      if (!parseVectorD(node[i], "joint_values", joint_values)){
		ROS_ERROR("Can't read joint_values");
	      }
	      else{
		sensor_msgs::JointState js;
		for(int j=0;j<joint_values.size();j++){
		  js.position.push_back(joint_values[j]);
		}
		joint_trajectory_data.push_back(js);
		write_joint_scene(js.position);
	      }
	    }
	  }
      }

    motion_client_ = new RobotJointValuesClient(motion_server_name_.c_str(), true);
    trigger_client_ = nh_.serviceClient<std_srvs::Trigger>(trigger_server_name_.c_str());

    // advertise services
    move_to_start_srv  = nh_.advertiseService( "JTrajMoveStart",   &JointTraj::MoveStartCallBack, this);
    move_to_next_srv   = nh_.advertiseService( "JTrajMoveNext",    &JointTraj::MoveNextCallBack, this);
    move_to_prev_srv   = nh_.advertiseService( "JTrajMovePrev",    &JointTraj::MovePrevCallBack, this);
    move_to_end_srv    = nh_.advertiseService( "JTrajMoveEnd",     &JointTraj::MoveEndCallBack, this);
    caputure_scene_srv = nh_.advertiseService( "JTrajCapture",     &JointTraj::CaptureCallBack, this);
    execute_srv        = nh_.advertiseService( "JTrajExecute",     &JointTraj::ExecuteCallBack, this);
    execute_wcall_srv  = nh_.advertiseService( "JTrajExecuteWCall", &JointTraj::ExecuteWTriggerCallBack, this);
  }; // end of constructor

  // destructor
  ~JointTraj()
  {
    ROS_ERROR("closing file");
    ofile_.close();
    delete (motion_client_);
  } // end of destructor


private:
  ros::NodeHandle nh_;

  std::string in_file_name_;
  std::string out_file_name_;
  std::string motion_server_name_;
  std::string trigger_server_name_;
  std::string joints_topic_name_;
  double pause_time_;
  std::ofstream ofile_;

  std::vector< sensor_msgs::JointState > joint_trajectory_data;
  int current_index_;
  ros::ServiceServer move_to_start_srv;
  ros::ServiceServer move_to_next_srv;
  ros::ServiceServer move_to_prev_srv;
  ros::ServiceServer move_to_end_srv;
  ros::ServiceServer caputure_scene_srv;
  ros::ServiceServer execute_srv;
  ros::ServiceServer execute_wcall_srv;
  RobotJointValuesClient* motion_client_;
  ros::ServiceClient trigger_client_;
  std_srvs::Trigger srv_;

  // private member functions
  // helper function for writing the yaml file containing the joint trajectory
  void write_header() 
  {
    ofile_ << "---\nposes:\n";
  } // end of write_header()

  // helper function for writing yaml file containing joint trajectory
  void write_joint_scene( const std::vector<double>& joints) 
  {
    ROS_INFO("writing new joint pose");

    ofile_ << "-\n";
    ofile_ << "     joint_values:\n";
    for (size_t i = 0; i < joints.size(); ++i)
    {
      ofile_ << "        - " << joints[i] << '\n';
    }
    ofile_.flush();
    return;
  } // end of write_joint_scene()

  // move to pose with given index into list of joint poses
  bool moveIndex(int goal_index)
  {
    if(goal_index > (int) joint_trajectory_data.size()){
      ROS_ERROR("goal_index = %d but trajectory size = %d", goal_index, (int) joint_trajectory_data.size());
      return(false);
    }      
    industrial_extrinsic_cal::robot_joint_values_triggerGoal goal;
    motion_client_->waitForServer();


    goal.joint_values.clear();
    for (int j = 0; j < (int)joint_trajectory_data[goal_index].position.size(); j++)
      {
	goal.joint_values.push_back(joint_trajectory_data[goal_index].position[j]);
      }
    ROS_INFO("SENDING GOAL");
    motion_client_->sendGoal(goal);
    do
      {
	motion_client_->waitForResult(ros::Duration(5.0));
	ROS_INFO("Current State: %s", motion_client_->getState().toString().c_str());
      } while (motion_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
	       motion_client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
    if(pause_time_>0.01){
      sleep(pause_time_);
    }
    return(true);
  }
  
  bool CaptureCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    sensor_msgs::JointStateConstPtr joints =
      ros::topic::waitForMessage<sensor_msgs::JointState>(joints_topic_name_, ros::Duration(1.0));

    if (!joints)
      {
        ROS_ERROR("Could not capture joint states within capture timeframe");
	res.message = std::string("Couldn't save pose");
	res.success = false;
	return(true);
      }
    else
      {
	write_joint_scene( joints->position);
	joint_trajectory_data.push_back(*joints);
	char msg[100];
	sprintf(msg, "trajectory now has %d poses", (int) joint_trajectory_data.size());
	res.message = std::string("Saved pose") + msg;
	res.success = true;
	return(true);
      }
  };    

  // move to front of list
  bool MoveStartCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    if(moveIndex(0)){      
      current_index_ = 0;
      res.message = std::string("Moved to Start");
      res.success = true;
      return(true);
    }
    else{
      res.message = std::string("Could Not Moved to Start");
      res.success = false;
      return(true);
    }
  };// end of MoveStartCallBack

  // move to next point, cycle to front
  bool MoveNextCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    int  index = (current_index_+1) % (int)joint_trajectory_data.size();
    char index_msg[100];
    sprintf(index_msg, "%d", current_index_);// because stings are soooo easy to use just like cout<< and other breaks make by cs 
    if(moveIndex(index)){      
      current_index_ = index;
      res.message = std::string("Moved position index ") + index_msg;
      res.success = true;
      return(true);
    }
    else{
      res.message = std::string("Could Not Move to index ") + index_msg;
      res.success = false;
      return(true);
    }
  };// end of MoveNextCallBack
  
  // move to previous point, stop at front
    bool MovePrevCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
      int index = (current_index_-1);
      if(index<0) index = 0;
      char index_msg[100];
      sprintf(index_msg, "%d", current_index_);// because stings are soooo easy to use just like cout<< and other breaks make by cs 
      if(moveIndex(index)){      
	current_index_ = index;
	res.message = std::string("Moved position index") + index_msg;
	res.success = true;
	return(true);
      }
      else{
	res.message = std::string("Could Not Move to index")  + index_msg;
	res.success = false;
	return(true);
      }
    };// end of MovePrevCallBack

    bool MoveEndCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
      int index =  (int) joint_trajectory_data.size()-1;
      if(index<0) index = 0;
      char index_msg[100];
      sprintf(index_msg, "%d", current_index_);// because stings are soooo easy to use just like cout<< and other breaks make by cs 
      if(moveIndex(index)){      
	current_index_ = index;
	res.message = std::string("Moved position index")  + index_msg;
	res.success = true;
	return(true);
      }
      else{
	res.message = std::string("Could Not Move to index")  + index_msg;
	res.success = false;
	return(true);
      }
    };// end of MoveEndCallBack

  // move through all the poses
    bool ExecuteCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
      for(current_index_ = 0; current_index_ < (int) joint_trajectory_data.size(); current_index_++){
	if(!moveIndex(current_index_)) break;
      }
      if(current_index_ == (int) joint_trajectory_data.size())
	{
	  current_index_ = 0; // go back to the beginning
	  res.message = std::string("Execution Completed");
	  res.success = true;
	  return(true);
	}
      else{
	char index_msg[100];
	sprintf(index_msg, "%d", current_index_);// because stings are soooo easy to use just like cout<< and other breaks make by cs 
	res.message = std::string("Failed to execute complete path current_index_ = ") + index_msg;
	res.success = false;
	return(true);
      }
    } // end of executeCallBack()

  // move through all poses, stoping and calling service call at each pose
    bool ExecuteWTriggerCallBack( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
      // call trigger server at each pose
      for(current_index_ = 0; current_index_ < (int) joint_trajectory_data.size(); current_index_++){
	if(!moveIndex(current_index_)) break;
	trigger_client_.waitForExistence(ros::Duration(-1));
	if(!trigger_client_.call(srv_)){
	  ROS_ERROR("call to %s failed", trigger_server_name_.c_str());
	}
      }
      if(current_index_ == (int) joint_trajectory_data.size())
	{
	  current_index_ = 0; // go back to beginning
	  res.message = std::string("Execution Completed");
	  res.success = true;
	  return(true);
	}
      else{
	char index_msg[100];
	sprintf(index_msg, "%d data had %d poses", current_index_, (int) joint_trajectory_data.size());// because stings are soooo easy to use just like cout<< and other breaks make by cs 
	res.message = std::string("Failed to execute complete path current_index_ = ") + index_msg;
	res.success = false;
	return(true);
      }
    } // end of executeCallBack()
};// end of class JointTraj

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_traj");
  ros::NodeHandle nh("~");
  
  JointTraj traj(nh);

  // Main 50hz loop
  ros::Rate r(50); 
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Terminating joint_traj node");
  return 0;
}
