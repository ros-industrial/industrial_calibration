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
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>

class CameraObserverTrigger
{
public:
  CameraObserverTrigger(ros::NodeHandle nh);
  ~CameraObserverTrigger(){};
  bool executeCallBack(industrial_extrinsic_cal::camera_observer_trigger::Request& req,
                       industrial_extrinsic_cal::camera_observer_trigger::Response& res);

  bool userAcceptCallBack(industrial_extrinsic_cal::user_accept::Request& req,
                          industrial_extrinsic_cal::user_accept::Response& res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer trigger_server_;
  ros::ServiceServer accept_server_;
  int target_type_;
  int pattern_rows_;
  int pattern_cols_;
  bool user_accepted_;
};

CameraObserverTrigger::CameraObserverTrigger(ros::NodeHandle nh)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");
  if (!pnh.getParam("target_type", target_type_))
  {
    ROS_ERROR("Must set target_type");
  }
  if (!pnh.getParam("target_type", target_type_))
  {
    ROS_ERROR("Must set target_type");
  }

  if (target_type_ == 3 || target_type_ == 4)
  {
    // neither AR_TAGs nor balls targets need rows or columns
  }
  else
  {
    if (!pnh.getParam("pattern_rows", pattern_rows_))
    {
      ROS_ERROR("Must set pattern_rows");
    }
    if (!pnh.getParam("pattern_cols", pattern_cols_))
    {
      ROS_ERROR("Must set pattern_rows");
    }
  }
  trigger_server_ = nh_.advertiseService("ObserverTrigger", &CameraObserverTrigger::executeCallBack, this);
  accept_server_ = nh_.advertiseService("UserAccept", &CameraObserverTrigger::userAcceptCallBack, this);
}

bool CameraObserverTrigger::executeCallBack(industrial_extrinsic_cal::camera_observer_trigger::Request& req,
                                            industrial_extrinsic_cal::camera_observer_trigger::Response& res)
{
  ros::NodeHandle nh;
  int number_of_required_observations;
  industrial_extrinsic_cal::CameraObservations camera_observations;

  ROS_ERROR("%s", req.instructions.c_str());

  industrial_extrinsic_cal::ROSCameraObserver camera_observer(req.image_topic);
  camera_observer.clearObservations();
  camera_observer.clearTargets();
  boost::shared_ptr<industrial_extrinsic_cal::Target> target = boost::make_shared<industrial_extrinsic_cal::Target>();
  target->target_name_ = "junk";
  target->target_frame_ = "whocares";
  target->target_type_ = target_type_;
  if (target_type_ < 3)
  {  // neither ARTag nor Balls targets need these parameters
    target->circle_grid_parameters_.pattern_rows = pattern_rows_;
    target->circle_grid_parameters_.pattern_cols = pattern_cols_;
    target->circle_grid_parameters_.circle_diameter = 1.0;  // don't care here
    target->circle_grid_parameters_.is_symmetric = true;
    number_of_required_observations = pattern_rows_ * pattern_cols_;
  }
  if (target_type_ == 4)
  {
    number_of_required_observations = 3;
  }
  if (target_type_ == 3)
  {
    ROS_ERROR("AR_TAG does not work in this application");
  }
  industrial_extrinsic_cal::Roi roi;
  roi.x_min = req.roi_min_x;
  roi.y_min = req.roi_min_y;
  roi.x_max = req.roi_max_x;
  roi.y_max = req.roi_max_y;

  industrial_extrinsic_cal::Cost_function cost_type;

  camera_observer.clearTargets();
  camera_observer.addTarget(target, roi, cost_type);
  user_accepted_ = false;
  int num_observations = 0;
  while (num_observations != number_of_required_observations || !user_accepted_)
  {
    camera_observer.clearObservations();
    camera_observer.triggerCamera();
    while (!camera_observer.observationsDone())
      ;
    camera_observer.getObservations(camera_observations);
    num_observations = (int)camera_observations.size();
    ROS_DEBUG("camera observer found %d observations", (int)camera_observations.size());
  }
  return (true);
}
bool CameraObserverTrigger::userAcceptCallBack(industrial_extrinsic_cal::user_accept::Request& req,
                                               industrial_extrinsic_cal::user_accept::Response& res)
{
  user_accepted_ = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trigger_server");
  ros::NodeHandle node_handle;
  CameraObserverTrigger COT(node_handle);
  ros::AsyncSpinner spinner(2);  // use 2 threads, one for each service callback
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
