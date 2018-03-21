#include <ros/ros.h>

#include <string>
#include <fstream>

// ros includes
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <industrial_extrinsic_cal/robot_joint_values_triggerAction.h>
#include <yaml-cpp/yaml.h>
#include <industrial_extrinsic_cal/yaml_utils.h>

typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::robot_joint_values_triggerAction>
    RobotJointValuesClient;

using industrial_extrinsic_cal::yamlNodeFromFileName;
using industrial_extrinsic_cal::parseNode;
using industrial_extrinsic_cal::parseVectorD;
using industrial_extrinsic_cal::parseString;

class JointTraj
{
public:
  JointTraj(ros::NodeHandle nh, const std::string& ifn, const std::string& ofn, const std::string& msn): nh_(nh), ifile_(ifn), motion_server_name_(msn)
  {
    
    ofile_.open(ofn.c_str());
    if(!ofile_.is_open()){
      ROS_ERROR("can't open %s", ofn.c_str());
    }
    else{
      ROS_ERROR("opened %s", ofn.c_str());
    }
    write_header();

    YAML::Node traj_doc;
    if (!yamlNodeFromFileName(ifile_, traj_doc))
      {
	ROS_ERROR("Can't parse yaml file %s", ifile_.c_str());
      }
    else
      {
	ROS_INFO("input yaml file parsed");
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
    
    client_ = new RobotJointValuesClient(motion_server_name_.c_str(), true);
  }

  ~JointTraj()
  {
    ROS_ERROR("closing file");
    ofile_.close();
    delete (client_);
  }
  void execute()
  {
    industrial_extrinsic_cal::robot_joint_values_triggerGoal goal;
    client_->waitForServer();
    for(int i=0;i<joint_trajectory_data.size();i++)
      {
	goal.joint_values.clear();
	for (int j = 0; j < (int)joint_trajectory_data[i].position.size(); j++)
	  {
	    goal.joint_values.push_back(joint_trajectory_data[i].position[j]);
	  }
	ROS_INFO("SENDING GOAL");
	client_->sendGoal(goal);
	do
	  {
	    client_->waitForResult(ros::Duration(5.0));
	    ROS_INFO("Current State: %s", client_->getState().toString().c_str());
	  } while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
		   client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
      }	// end of for each trajectory point
  } // end of execute

  void goStart()
  {
    industrial_extrinsic_cal::robot_joint_values_triggerGoal goal;
    client_->waitForServer();
    // go back to beginning
    goal.joint_values.clear();
    for (int j = 0; j < (int)joint_trajectory_data[0].position.size(); j++)
      {
	goal.joint_values.push_back(joint_trajectory_data[0].position[j]);
      }
    ROS_INFO("SENDING GOAL");
    client_->sendGoal(goal);
    do
      {
	client_->waitForResult(ros::Duration(5.0));
	ROS_INFO("Current State: %s", client_->getState().toString().c_str());
      } while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
	       client_->getState() != actionlib::SimpleClientGoalState::ABORTED);

  } // end of goStart()

  void new_joint_scene(const sensor_msgs::JointState& state)
  {
    write_joint_scene( state.position);
    joint_trajectory_data.push_back(state);
  }
  ros::NodeHandle nh_;
  std::string ifile_;
  std::ofstream ofile_;
  std::string joints_topic_name_;
  std::vector< sensor_msgs::JointState > joint_trajectory_data;
  std::string motion_server_name_;
private:
  RobotJointValuesClient* client_;
  void write_header() 
  {
    ofile_ << "---\nposes:\n";
  }

  void write_joint_scene( const std::vector<double>& joints) 
  {
    ROS_ERROR("writing new joint pose");

    ofile_ << "-\n";
    ofile_ << "     joint_values:\n";
    for (size_t i = 0; i < joints.size(); ++i)
    {
      ofile_ << "        - " << joints[i] << '\n';
    }
    return;
  }
  
};// end of class JointTraj

int main(int argc, char** argv)
{
  std::string out_filename;
  std::string in_filename;
  std::string joints_topic_name;
  std::string motion_server_name;
  ros::init(argc, argv, "joint_traj");
  ros::NodeHandle nh("~");

  nh.param<std::string>("input_file", in_filename, "joint_traj.yaml");
  nh.param<std::string>("output_file", out_filename, "joint_traj2.yaml");
  nh.param<std::string>("joints_topic", joints_topic_name, "/joint_states");
  nh.param<std::string>("motion_server", motion_server_name, "rosRobotSceneTrigger_joint_values");
  
  // Load options from param server
  JointTraj traj(nh, in_filename, out_filename, motion_server_name);

  // Main loop
  while (ros::ok())
  {
    bool capture_scene = false;
    bool quit = false;
    bool execute = false;
    bool gostart = false;
    nh.getParam("capture_scene", capture_scene);
    nh.getParam("quit", quit);
    nh.getParam("execute", execute);
    nh.getParam("gostart", gostart);
    if (quit)
      break;
    else if (capture_scene)
    {
      nh.setParam("capture_scene", false);

      ROS_INFO("Attempting to capture robot state");
      // get robot joint state
      sensor_msgs::JointStateConstPtr joints =
          ros::topic::waitForMessage<sensor_msgs::JointState>(joints_topic_name, ros::Duration(1.0));

      if (!joints)
      {
        ROS_ERROR("Could not capture joint states within capture timeframe");
      }
      else
      {
          traj.new_joint_scene(*joints);
      }
    }
    else if (execute)
      {
      nh.setParam("execute", false);
	traj.execute();
      }
    else if(gostart){
      nh.setParam("gostart", false);
      traj.goStart();
    }
    ros::spinOnce();
  }
  ROS_INFO("Terminating jointTraj process");
  return 0;
}
