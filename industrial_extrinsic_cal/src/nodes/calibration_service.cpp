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
#include <industrial_extrinsic_cal/calibration_job_definition.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_extrinsic_cal/calibrationAction.h>
#include <industrial_extrinsic_cal/calibrate.h>
#include <industrial_extrinsic_cal/covariance.h>

using industrial_extrinsic_cal::CovarianceVariableRequest;

class CalibrationServiceNode
{
public:
  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::calibrationAction> CalibrationActionServer;

  explicit CalibrationServiceNode(const ros::NodeHandle& nh)
    : nh_(nh)
    , action_server_(nh_, "run_calibration", boost::bind(&CalibrationServiceNode::actionCallback, this, _1), false)
  {
    calibrated_ = false;
    std::string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");
    std::string camera_file;
    std::string target_file;
    std::string caljob_file;
    std::string yaml_file_path = ros::package::getPath("industrial_extrinsic_cal") + "/yaml/";
    bool post_proc_on = false;
    std::string observation_data_file;
    priv_nh.getParam("yaml_file_path", yaml_file_path);
    priv_nh.getParam("camera_file", camera_file);
    priv_nh.getParam("target_file", target_file);
    priv_nh.getParam("cal_job_file", caljob_file);
    priv_nh.getParam("post_proc_on", post_proc_on);
    priv_nh.getParam("observation_data_file", observation_data_file);
    if (!priv_nh.getParam("results_file", results_file_))
    {
      results_file_ = yaml_file_path + "results.launch";
    }
    ROS_INFO("yaml_file_path: %s", yaml_file_path.c_str());
    ROS_INFO("camera_file: %s", camera_file.c_str());
    ROS_INFO("target_file: %s", target_file.c_str());
    ROS_INFO("cal_job_file: %s", caljob_file.c_str());
    ROS_INFO("results_file: %s", results_file_.c_str());

    cal_job_ = new industrial_extrinsic_cal::CalibrationJob(yaml_file_path + camera_file, yaml_file_path + target_file,
                                                            yaml_file_path + caljob_file);

    if (post_proc_on) cal_job_->postProcessingOn(observation_data_file);
    if (cal_job_->load())
    {
      ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
    }
    action_server_.start();
  };

  ~CalibrationServiceNode()
  {
    delete (cal_job_);
  }
  bool callback(industrial_extrinsic_cal::calibrate::Request& req, industrial_extrinsic_cal::calibrate::Response& resp);
  bool actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal);
  bool is_calibrated()
  {
    return (calibrated_);
  };
  bool covarianceCallback(industrial_extrinsic_cal::covariance::Request& req,
                          industrial_extrinsic_cal::covariance::Response& res);

private:
  ros::NodeHandle nh_;
  bool calibrated_;
  industrial_extrinsic_cal::CalibrationJob* cal_job_;
  CalibrationActionServer action_server_;
  std::string results_file_;
};

bool CalibrationServiceNode::covarianceCallback(industrial_extrinsic_cal::covariance::Request& req,
                                                industrial_extrinsic_cal::covariance::Response& res)
{
  std::vector<CovarianceVariableRequest> requests;
  CovarianceVariableRequest request1, request2;

  request1.request_type = industrial_extrinsic_cal::intToCovRequest(req.request_type1);
  request1.object_name = req.block_name1;
  request1.scene_id = req.scene_id1;
  requests.push_back(request1);

  request2.request_type = industrial_extrinsic_cal::intToCovRequest(req.request_type2);
  request2.object_name = req.block_name2;
  request2.scene_id = req.scene_id2;
  requests.push_back(request2);
  std::string file_name = req.file_name;
  bool ret = cal_job_->computeCovariance(requests, file_name);
  // set results and return
  res.result = 1;  // just a placeholder
  return (ret);
}

bool CalibrationServiceNode::callback(industrial_extrinsic_cal::calibrate::Request& req,
                                      industrial_extrinsic_cal::calibrate::Response& res)
{
  // Display initial state
  ROS_INFO("State prior to optimization");
  cal_job_->show();

  // Run observations and subsequent optimization
  ROS_INFO("RUNNING");
  if (cal_job_->run())
  {
    res.cost_per_observation = cal_job_->finalCostPerObservation();
    ROS_INFO("Calibration Sucessful. Initial cost per observation = %lf final cost per observation %lf",
             cal_job_->initialCostPerObservation(), cal_job_->finalCostPerObservation());
    if (cal_job_->finalCostPerObservation() <= req.allowable_cost_per_observation)
    {
      calibrated_ = true;
      if (!cal_job_->store(results_file_))
      {
        ROS_ERROR_STREAM(" Trouble storing calibration job optimization results ");
      }
    }
    else
    {
      ROS_ERROR("Calibration ran successfully, but error was larger than allowed by caller");
      calibrated_ = false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Calibration job failed");
    return (false);
  }

  // Show Results
  cal_job_->show();

  return (calibrated_);
}

bool CalibrationServiceNode::actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal)
{
  industrial_extrinsic_cal::calibrate::Request request;
  industrial_extrinsic_cal::calibrate::Response response;
  request.allowable_cost_per_observation = goal->allowable_cost_per_observation;
  if (callback(request, response))
  {
    action_server_.setSucceeded();
    return (true);
  }
  action_server_.setAborted();
  return (false);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_service_node");
  ros::NodeHandle nh;
  CalibrationServiceNode cal_service_node(nh);

  ros::ServiceServer cal_service =
      nh.advertiseService("calibration_service", &CalibrationServiceNode::callback, &cal_service_node);
  ros::ServiceServer cov_service =
      nh.advertiseService("covariance_service", &CalibrationServiceNode::covarianceCallback, &cal_service_node);

  ros::spin();
}
