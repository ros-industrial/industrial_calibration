/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ROS_SCENE_TRIGGER_H_
#define ROS_SCENE_TRIGGER_H_

#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <industrial_extrinsic_cal/scene_trigger.h>
#include <industrial_extrinsic_cal/manual_triggerAction.h>

namespace industrial_extrinsic_cal
{
class ROSParamSceneTrigger : public SceneTrigger
{
public:
  /*! \brief Constructor,
   */
  ROSParamSceneTrigger(std::string parameter_name)
  {
    parameter_name_ = parameter_name;
    nh_.setParam(parameter_name_.c_str(), false);
  };
  /*! \brief Destructor
   */
  ~ROSParamSceneTrigger(){};
  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    ROS_ERROR("ROSParamSceneTrigger: waiting for %s to be true", parameter_name_.c_str());
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

typedef actionlib::SimpleActionClient<industrial_extrinsic_cal::manual_triggerAction> Client;

class ROSActionServerSceneTrigger : public SceneTrigger
{
public:
  /*! \brief Constructor,
   */
  ROSActionServerSceneTrigger(std::string server_name, std::string action_message)
  {
    server_name_ = server_name;
    action_message_ = action_message;
  };
  /*! \brief Destructor
   */
  ~ROSActionServerSceneTrigger(){};
  /*! \brief Initiates and waits for trigger to finish
   */
  bool waitForTrigger()
  {
    ROS_ERROR("ROSActionServerSceneTrigger: waiting for trigger server %s to complete ", server_name_.c_str());
    Client client(server_name_.c_str(), true);
    client.waitForServer();
    industrial_extrinsic_cal::manual_triggerGoal goal;
    goal.display_message = action_message_;
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    while (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Current State: %s", client.getState().toString().c_str());
      client.waitForResult(ros::Duration(5.0));
    }
    return (true);
  };

private:
  ros::NodeHandle nh_;
  std::string server_name_;
  std::string action_message_;
};
}  // end of namespace
#endif
