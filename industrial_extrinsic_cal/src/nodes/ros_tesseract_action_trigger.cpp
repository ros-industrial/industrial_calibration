/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ros/service_server.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <industrial_extrinsic_cal/robot_joint_values_triggerAction.h>  // one of the ros action messages
#include <industrial_extrinsic_cal/robot_pose_triggerAction.h>          // the other ros action message
#include <industrial_robot_client/joint_trajectory_action.h>

#include <trajopt_utils/logging.hpp>
#include <trajopt/problem_description.hpp>

#include <control_msgs/FollowJointTrajectoryActionGoal.h> // a client for streaming exection of a joint trajectory
#include <sensor_msgs/JointState.h>

#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_motion_planners/trajopt/trajopt_freespace_planner.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/utils.h>


class TesseractActionServer
{
  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::robot_joint_values_triggerAction> JointValuesServer;
  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::robot_pose_triggerAction> PoseServer;

protected:

  ros::NodeHandle nh_;
  std::string joint_action_server_name_;
  std::string pose_action_server_name_;
  std::string robot_joint_state_topic_;
  std::string manipulator_name_;
  JointValuesServer jvs_;
  PoseServer ps_;
  trajectory_msgs::JointTrajectory robot_traj_;

  industrial_extrinsic_cal::robot_joint_values_triggerResult joint_result_;
  industrial_extrinsic_cal::robot_joint_values_triggerFeedback joint_feedback_;

  industrial_extrinsic_cal::robot_pose_triggerResult pose_result_;
  industrial_extrinsic_cal::robot_pose_triggerFeedback pose_feedback_;

  tesseract_monitoring::EnvironmentMonitorPtr env_monitor_;
  tesseract_motion_planners::TrajOptFreespacePlanner freespace_trajopt_planner_;
  
public:
  TesseractActionServer(ros::NodeHandle& nh, std::string joint_action_server_name, std::string pose_action_server_name)
    : nh_(nh),
      joint_action_server_name_(joint_action_server_name),
      pose_action_server_name_(pose_action_server_name),
      jvs_(nh_, joint_action_server_name, boost::bind(&TesseractActionServer::joint_move, this, _1), false),
      ps_( nh_, pose_action_server_name, boost::bind(&TesseractActionServer::pose_move, this, _1), false)
  {
    // Get ROS params
    std::string robot_description, srdf_xml_string;
    if(!nh_.getParam("robot_description", robot_description)){
      ROS_ERROR("For TesseractActionServer, must set robot_description parameter");
    }
    if(!nh_.getParam("manipulator_name", manipulator_name_))  manipulator_name_ = "manipulator";
    if(!nh_.getParam("robot_joint_state_topic", robot_joint_state_topic_)) robot_joint_state_topic_ = "/joint_states";
    
    std::string descrete_plugin, continuous_plugin;
    nh_.param<std::string>("descrete_plugin", descrete_plugin, "");
    nh_.param<std::string>("continuous_plugin", continuous_plugin, "");

    
    // TODO fix UNKNOWN to be some name original code showed RIPL and I don't know why
    // set up environment
    env_monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(robot_description, "UNKOWN", "", "");
    env_monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_GEOMETRY);
    env_monitor_->startStateMonitor(robot_joint_state_topic_);
    
    jvs_.start();
    
    ROS_INFO("Ready to move the robot.");
  }
  
  bool pose_move(const industrial_extrinsic_cal::robot_pose_triggerGoalConstPtr& goal)
  {
    tesseract_motion_planners::PlannerResponse planner_response;
    tesseract_motion_planners::TrajOptFreespacePlannerConfig config;
    // find start_waypoint by listening to joint states
    boost::shared_ptr<sensor_msgs::JointState const> joint_start;
    joint_start = ros::topic::waitForMessage<sensor_msgs::JointState>(robot_joint_state_topic_, nh_, ros::Duration(1));

    // configure for planning
    config.start_waypoint_ = std::make_shared<tesseract_motion_planners::JointWaypoint>(joint_start->position,joint_start->name);
    Eigen::Vector3d position(goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
    Eigen::Quaterniond quat(goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
    config.end_waypoint_ = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(position,quat);
    config.num_steps_ = 20;
    config.tesseract_ = env_monitor_->getTesseractConst();
    config.link_ = config.tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator_name_)->getTipLinkName();
    config.tcp_ = Eigen::Isometry3d::Identity();
    config.manipulator_ = manipulator_name_;
    config.init_type_ = trajopt::InitInfo::STATIONARY;
    config.collision_check_ = true;
    config.collision_continuous_ = false;

    // solve/plan for a joint trajectory
    freespace_trajopt_planner_.clear();
    freespace_trajopt_planner_.setConfiguration(config);
    freespace_trajopt_planner_.solve(planner_response);

    if (!planner_response.status)
      {
	ROS_ERROR("Free Space Plan Unsuccessful");
	return false;
      }


    // a planner response has a tesseract_common::JointTrajectory joint_trajectory
    // a planner response has a tesseract_common::StatusCode  status
    // trajectory_msgs::JointTrajectory robot_traj_ has a Header header
    // trajectory_msgs::JointTrajectory robot_traj_ has a string[] joint_names
    // trajectory_msgs::JointTrajectory robot_traj_ has a JointTrajectory[] points
    // here's the signature: inline void toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj)
    tesseract_rosutils::toMsg(robot_traj_, planner_response.joint_trajectory);

    // stream to robot
    if (!this->motion_streaming(robot_traj_))
      {
	ROS_ERROR("Motion Failed");
	return false;
      }

    ROS_INFO("%s: Succeeded", joint_action_server_name_.c_str());

    // set the action state to succeeded
    pose_result_.result = 1;
    ps_.setSucceeded(pose_result_);
      
    return true;

  }
  bool joint_move(const industrial_extrinsic_cal::robot_joint_values_triggerGoalConstPtr& goal)
  {
    tesseract_motion_planners::PlannerResponse planner_response;
    tesseract_motion_planners::TrajOptFreespacePlannerConfig config;

    // find start_waypoint by listening to joint states
    boost::shared_ptr<sensor_msgs::JointState const> joint_start;
    joint_start = ros::topic::waitForMessage<sensor_msgs::JointState>(robot_joint_state_topic_, nh_, ros::Duration(1));

    if(joint_start == NULL || (int) joint_start->position.size() != (int) goal->joint_values.size()){
      ROS_ERROR("# of joints of goal != # of joints of robot %d != %d", (int) joint_start->position.size(), (int) goal->joint_values.size());
      return(false);
    }

    // configure for planning
    config.start_waypoint_ = std::make_shared<tesseract_motion_planners::JointWaypoint>(joint_start->position,joint_start->name);
    config.end_waypoint_ = std::make_shared<tesseract_motion_planners::JointWaypoint>(goal->joint_values, joint_start->name);
    config.num_steps_ = 20;
    config.tesseract_ = env_monitor_->getTesseractConst();
    config.link_ = config.tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator_name_)->getTipLinkName();
    config.tcp_ = Eigen::Isometry3d::Identity();
    config.manipulator_ = manipulator_name_;
    config.init_type_ = trajopt::InitInfo::STATIONARY;
    config.collision_check_ = true;
    config.collision_continuous_ = false;

    // solve/plan for a joint trajectory
    freespace_trajopt_planner_.clear();
    freespace_trajopt_planner_.setConfiguration(config);
    freespace_trajopt_planner_.solve(planner_response);

    if (!planner_response.status)
      {
	ROS_ERROR("Free Space Plan Unsuccessful");
	return false;
      }


    // a planner response has a tesseract_common::JointTrajectory joint_trajectory
    // a planner response has a tesseract_common::StatusCode  status
    // trajectory_msgs::JointTrajectory robot_traj_ has a Header header
    // trajectory_msgs::JointTrajectory robot_traj_ has a string[] joint_names
    // trajectory_msgs::JointTrajectory robot_traj_ has a JointTrajectory[] points
    // here's the signature: inline void toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj)
    tesseract_rosutils::toMsg(robot_traj_, planner_response.joint_trajectory);

    // stream to robot
    if (!this->motion_streaming(robot_traj_))
      {
	ROS_ERROR("Motion Failed");
	return false;
      }

    ROS_INFO("%s: Succeeded", joint_action_server_name_.c_str());

    // set the action state to succeeded
    joint_result_.result = 1;
    jvs_.setSucceeded(joint_result_);
      
    return true;
      
  }

  bool motion_streaming(trajectory_msgs::JointTrajectory robot_traj)
  {
    //////////////////////////////
    /// Robot Motion Streaming ///
    //////////////////////////////

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = robot_traj;

    ac.sendGoal(goal);
    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
      {
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
      }
    else
      {
	ROS_INFO("Action did not finish before the time out.");
	return false;
      }

    return true;

  }


};



int main(int argc, char **argv)
{
  //////////////////////
  /// Initialization ///
  //////////////////////

  ros::init(argc, argv, "ros_tesseract_action_trigger");
  ros::NodeHandle nh;
  ros::AsyncSpinner async_spinner(3);
  async_spinner.start();
  
  ROS_INFO("Getting ready to move the robot.");

  std::string joint_server("tesseract_joint_motion");
  std::string pose_server("tesseract_pose_motion");

  ROS_ERROR("joint_action server = %s pose action server = %s", joint_server.c_str(), pose_server.c_str());
  TesseractActionServer mover(nh, joint_server, pose_server);
  
  // Set Log Level
  util::gLogLevel = util::LevelInfo;


  ros::waitForShutdown();

  return 0;
}
