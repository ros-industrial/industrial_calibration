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

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_extrinsic_cal/manual_triggerAction.h>

typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::manual_triggerAction> Server;

void execute(const industrial_extrinsic_cal::manual_triggerGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  ROS_ERROR("Scene Action Trigger is waiting for you to type: rosparam set test_scene_trigger true");
  bool test_scene_trigger_bool = false;
  ros::NodeHandle nh;
  nh.setParam("test_scene_trigger", false);
  while (test_scene_trigger_bool == false)
  {
    nh.getParam("test_scene_trigger", test_scene_trigger_bool);
  }
  ROS_ERROR("Scene Action Trigger has executed successfully");
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scene trigger server");
  ros::NodeHandle n;
  Server server(n, "rosSceneTrigger", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
