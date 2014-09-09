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

#ifndef ROS_TRIGGER_H_
#define ROS_TRIGGER_H_

#include <industrial_extrinsic_cal/trigger.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/manual_triggerAction.h>
#include <industrial_extrinsic_cal/robot_joint_values_triggerAction.h>
#include <industrial_extrinsic_cal/robot_pose_triggerAction.h>

namespace industrial_extrinsic_cal
{
  class ROSParamTrigger : public Trigger
  {
  public:
    /*! \brief Constructor,
     */
    ROSParamTrigger(const std::string & parameter_name) 
      {
	parameter_name_ = parameter_name;  
	nh_.setParam(parameter_name_.c_str(),false);
      };
    /*! \brief Destructor
     */
    ~ROSParamTrigger(){};
    /*! \brief Initiates and waits for trigger to finish
     */
    bool waitForTrigger()
    {
      ROS_ERROR("ROSParamTrigger:  waiting for %s to be true",parameter_name_.c_str());
      bool pval = false;
      while(!pval){
	nh_.getParam(parameter_name_.c_str(),pval);
      }
      nh_.setParam(parameter_name_.c_str(),false);
      return(true);
    };
  private: 
    ros::NodeHandle nh_;
    std::string parameter_name_;
  };

  typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::manual_triggerAction> ManualClient;

  class ROSActionServerTrigger : public Trigger
  {
  public:
    /*! \brief Constructor,
     */
    ROSActionServerTrigger(const std::string & server_name, const  std::string  & action_message) 
      {
	server_name_ = server_name;  
	action_message_ = action_message;  
	client_ = new ManualClient(server_name_.c_str(),true);
      };

    /*! \brief Destructor
     */
    ~ROSActionServerTrigger(){};

    /*! \brief Initiates and waits for trigger to finish
     */
    bool waitForTrigger()
    {
      ROS_INFO("ROSActionServerTrigger: waiting for trigger server %s to complete ",server_name_.c_str());
      client_->waitForServer();
      industrial_extrinsic_cal::manual_triggerGoal goal;
      goal.display_message = action_message_;
      client_->sendGoal(goal);
      do{
	client_->waitForResult(ros::Duration(5.0));
	ROS_INFO("Current State: %s", client_->getState().toString().c_str());
      } while(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
	      && client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
      return(true);  /**< TODO implement a timeout, cancels action and with returns false*/
    };
  private: 
    ManualClient *client_;
    ros::NodeHandle nh_;	/**< node handle */
    std::string server_name_;	/**< name of server */
    std::string action_message_; /**< message sent to action server, often displayed by that server */
  };


    typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::robot_joint_values_triggerAction> RobotJointValuesClient;

  class ROSRobotJointValuesActionServerTrigger : public Trigger
  {
  public:
    /*! \brief Constructor,
     */
    ROSRobotJointValuesActionServerTrigger(const std::string & server_name, const  std::vector<double> &joint_values) 
      {
	server_name_ = server_name;  
	joint_values_.clear();
	for(int i=0; i< (int)joint_values.size(); i++){
	  joint_values_.push_back(joint_values[i]);
	}
	client_ = new RobotJointValuesClient(server_name_.c_str(),true);
      };

    /*! \brief Destructor
     */
    ~ROSRobotJointValuesActionServerTrigger(){
      delete(client_);
    };

    /*! \brief Initiates and waits for trigger to finish
     */
    bool waitForTrigger()
    {
      ROS_INFO("ROSRobotJointValuesActionServerTrigger: waiting for trigger server %s to complete ",server_name_.c_str());
      client_->waitForServer();
      industrial_extrinsic_cal::robot_joint_values_triggerGoal goal;
      goal.joint_values.clear();
      for(int i=0; i<(int)joint_values_.size();i++){
	goal.joint_values.push_back(joint_values_[i]);
      }
      ROS_INFO("SENDING GOAL");
      client_->sendGoal(goal);
      do{
	client_->waitForResult(ros::Duration(5.0));
	ROS_INFO("Current State: %s", client_->getState().toString().c_str());
      } while(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
	      && client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
      return(true);  /**< TODO implement a timeout, cancels action and with returns false*/
    };
  private: 
    RobotJointValuesClient *client_;
    ros::NodeHandle nh_;	/**< node handle */
    std::string server_name_;	/**< name of server */
    std::vector<double> joint_values_; /**< joint values */
  };

    typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::robot_pose_triggerAction> Robot_Pose_Client;
 

  class ROSRobotPoseActionServerTrigger : public Trigger
  {
  public:
    /*! \brief Constructor,
     */
    ROSRobotPoseActionServerTrigger(const std::string & server_name, const  geometry_msgs::Pose pose) 
      {
	server_name_ = server_name;  
	joint_values_.clear();
	pose_ = pose;
	client_ = new Robot_Pose_Client(server_name_.c_str(),true);
      };

    /*! \brief Destructor
     */
    ~ROSRobotPoseActionServerTrigger(){};

    /*! \brief Initiates and waits for trigger to finish
     */
    bool waitForTrigger()
    {
      ROS_INFO("ROSRobotPoseActionServerTrigger: waiting for trigger server %s to complete ",server_name_.c_str());
      client_->waitForServer();
      industrial_extrinsic_cal::robot_pose_triggerGoal goal;
      goal.pose = pose_;
      client_->sendGoal(goal);
      do{
	client_->waitForResult(ros::Duration(5.0));
	ROS_INFO("Current State: %s", client_->getState().toString().c_str());
      } while(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
	      && client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
      return(true);  /**< TODO implement a timeout, cancels action and with returns false*/
    };
  private: 
    Robot_Pose_Client *client_;
    ros::NodeHandle nh_;	/**< node handle */
    std::string server_name_;	/**< name of server */
    geometry_msgs::Pose pose_; /**< pose of robot */
    std::vector<double> joint_values_; /**< joint values */
  };

}// end of namespace

#endif
