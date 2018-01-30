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
#include <industrial_extrinsic_cal/camera_observer_trigger.h>

namespace industrial_extrinsic_cal
{
class ROSParamTrigger : public Trigger
{
public:
  /*! \brief Constructor,
   * @param parameter_name, name of trigger
   */
  ROSParamTrigger(const std::string& parameter_name)
  {
    parameter_name_ = parameter_name;
    nh_.setParam(parameter_name_.c_str(), false);
  };
  /*! \brief Destructor
   */
  ~ROSParamTrigger(){};
  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    ROS_ERROR("ROSParamTrigger:  waiting for %s to be true", parameter_name_.c_str());
    bool pval = false;
    while (!pval)
    {
      nh_.getParam(parameter_name_.c_str(), pval);
    }
    nh_.setParam(parameter_name_.c_str(), false);
    return (true);
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
   * @param server_name, name of tigger service
   * @param action_message message to display
   */
  ROSActionServerTrigger(const std::string& server_name, const std::string& action_message)
  {
    server_name_ = server_name;
    action_message_ = action_message;
    client_ = new ManualClient(server_name_.c_str(), true);
  };

  /*! \brief Destructor
   */
  ~ROSActionServerTrigger(){};

  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    ROS_INFO("ROSActionServerTrigger: waiting for trigger server %s to complete ", server_name_.c_str());
    client_->waitForServer();
    industrial_extrinsic_cal::manual_triggerGoal goal;
    goal.display_message = action_message_;
    client_->sendGoal(goal);
    do
    {
      client_->waitForResult(ros::Duration(5.0));
      ROS_INFO("Current State: %s", client_->getState().toString().c_str());
    } while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
             client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
    return (true); /**< TODO implement a timeout, cancels action and with returns false*/
  };

private:
  ManualClient* client_;
  ros::NodeHandle nh_;         /**< node handle */
  std::string server_name_;    /**< name of server */
  std::string action_message_; /**< message sent to action server, often displayed by that server */
};

typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::robot_joint_values_triggerAction>
    RobotJointValuesClient;

class ROSRobotJointValuesActionServerTrigger : public Trigger
{
public:
  /*! \brief Constructor,
   *  @param server name , name of server
   *  @param joint_values, vector of joint values describing the pose of the robot for the trigger
   */
  ROSRobotJointValuesActionServerTrigger(const std::string& server_name, const std::vector<double>& joint_values)
  {
    server_name_ = server_name;
    joint_values_.clear();
    for (int i = 0; i < (int)joint_values.size(); i++)
    {
      joint_values_.push_back(joint_values[i]);
    }
    client_ = new RobotJointValuesClient(server_name_.c_str(), true);
  };

  /*! \brief Destructor
   */
  ~ROSRobotJointValuesActionServerTrigger()
  {
    delete (client_);
  };

  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    ROS_INFO("ROSRobotJointValuesActionServerTrigger: waiting for trigger server %s to complete ",
             server_name_.c_str());
    client_->waitForServer();
    industrial_extrinsic_cal::robot_joint_values_triggerGoal goal;
    goal.joint_values.clear();
    for (int i = 0; i < (int)joint_values_.size(); i++)
    {
      goal.joint_values.push_back(joint_values_[i]);
    }
    ROS_INFO("SENDING GOAL");
    client_->sendGoal(goal);
    do
    {
      client_->waitForResult(ros::Duration(5.0));
      ROS_INFO("Current State: %s", client_->getState().toString().c_str());
    } while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
             client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
    return (true); /**< TODO implement a timeout, cancels action and with returns false*/
  };

private:
  RobotJointValuesClient* client_;
  ros::NodeHandle nh_;               /**< node handle */
  std::string server_name_;          /**< name of server */
  std::vector<double> joint_values_; /**< joint values */
};

typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::robot_pose_triggerAction> Robot_Pose_Client;

class ROSRobotPoseActionServerTrigger : public Trigger
{
public:
  /*! \brief Constructor,
   *   @param server_name name of the service
   *   @param pose the pose of the robot for the trigger
   */
  ROSRobotPoseActionServerTrigger(const std::string& server_name, const Pose6d pose)
  {
    server_name_ = server_name;
    joint_values_.clear();
    pose_.position.x = pose.x;
    pose_.position.y = pose.y;
    pose_.position.z = pose.z;
    double qx, qy, qz, qw;
    pose.getQuaternion(qx, qy, qz, qw);
    pose_.orientation.x = qx;
    pose_.orientation.y = qy;
    pose_.orientation.z = qz;
    pose_.orientation.w = qw;
    client_ = new Robot_Pose_Client(server_name_.c_str(), true);
  };

  /*! \brief Destructor
   */
  ~ROSRobotPoseActionServerTrigger(){};

  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    ROS_INFO("ROSRobotPoseActionServerTrigger: waiting for trigger server %s to complete ", server_name_.c_str());
    client_->waitForServer();
    industrial_extrinsic_cal::robot_pose_triggerGoal goal;
    goal.pose = pose_;
    client_->sendGoal(goal);
    do
    {
      client_->waitForResult(ros::Duration(5.0));
      ROS_INFO("Current State: %s", client_->getState().toString().c_str());
    } while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
             client_->getState() != actionlib::SimpleClientGoalState::ABORTED);
    return (true); /**< TODO implement a timeout, cancels action and with returns false*/
  };

private:
  Robot_Pose_Client* client_;
  ros::NodeHandle nh_;               /**< node handle */
  std::string server_name_;          /**< name of server */
  geometry_msgs::Pose pose_;         /**< pose of robot */
  std::vector<double> joint_values_; /**< joint values */
};

class ROSCameraObserverTrigger : public Trigger
{
public:
  /*! \brief Constructor,
   *   @param service_name name of service
   *    @param instructions a message for the user
   *    @param image_topic the image to use and display to user to accept the trigger
   *    @param roi the region of interest in the image displayed to the user
   */
  ROSCameraObserverTrigger(const std::string& service_name, const std::string& instructions,
                           const std::string& image_topic, const Roi& roi)
  {
    nh_ = new ros::NodeHandle;
    service_name_ = service_name;
    instructions_ = instructions;
    image_topic_ = image_topic;
    client_ = nh_->serviceClient<industrial_extrinsic_cal::camera_observer_trigger>(service_name_.c_str());
    roi_ = roi;
  };

  /*! \brief Destructor
   */
  ~ROSCameraObserverTrigger(){};

  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    industrial_extrinsic_cal::camera_observer_trigger::Request request;
    industrial_extrinsic_cal::camera_observer_trigger::Response response;

    ROS_INFO("ROSRobotPoseActionServerTrigger: waiting for trigger server %s to complete ", service_name_.c_str());
    request.image_topic = image_topic_;
    request.instructions = instructions_;
    request.roi_min_x = roi_.x_min;
    request.roi_max_x = roi_.x_max;
    request.roi_min_y = roi_.y_min;
    request.roi_max_y = roi_.y_max;

    if (!client_.call(request, response))
    {
      ROS_ERROR("Trigger failed");
    }
    return (true); /**< TODO implement a timeout, cancels action and with returns false*/
  };

private:
  ros::ServiceClient client_;         /**< the client to call supplying this trigger */
  ros::NodeHandle* nh_;               /**< node handle */
  std::string service_name_;          /**< name of server */
  std::string instructions_;          /**< instructions to display to user */
  std::string image_topic_;           /**< image from this ros topic used to find target within the given roi */
  industrial_extrinsic_cal::Roi roi_; /**< region to find the target */
};

}  // end of namespace

#endif
