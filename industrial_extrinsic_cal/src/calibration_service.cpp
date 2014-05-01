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


#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <industrial_extrinsic_cal/calibration_job_definition.h>
class CalibrationServiceNode
{
public:
  explicit CalibrationServiceNode(const ros::NodeHandle& nh):
    nh_(nh)
  {
    calibrated_ = false;
    std::string nn = ros::this_node::getName();
    ros::NodeHandle priv_nh("~");
    std::string camera_file;
    std::string target_file;
    std::string caljob_file;
    std::string ros_package_name;
    std::string launch_file_name;
    priv_nh.getParam("camera_file", camera_file);
    priv_nh.getParam("target_file", target_file);
    priv_nh.getParam("cal_job_file", caljob_file);
    priv_nh.getParam("store_results_package_name", ros_package_name);
    priv_nh.getParam("store_results_file_name", launch_file_name);
    
    std::string path = ros::package::getPath("industrial_extrinsic_cal");
    std::string file_path=path+"/yaml/";
    ROS_INFO("path: %s",file_path.c_str());
    ROS_INFO("camera_file: %s",camera_file.c_str());
    ROS_INFO("target_file: %s",target_file.c_str());
    ROS_INFO("cal_job_file: %s",caljob_file.c_str());
    ROS_INFO("store results: %s",ros_package_name.c_str());
    ROS_INFO("launch_file_name: %s",launch_file_name.c_str());
    
    cal_job_ = new industrial_extrinsic_cal::CalibrationJob(file_path+camera_file, file_path+target_file, file_path+caljob_file);
  
    if (cal_job_->load())
      {
	ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
      }
    
  };

  ~CalibrationServiceNode()
  {
    delete( cal_job_);
  }
  bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool is_calibrated(){return(calibrated_);};
private:
  ros::NodeHandle nh_;
  bool calibrated_;
  industrial_extrinsic_cal::CalibrationJob * cal_job_;
};

bool CalibrationServiceNode::callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  
// Display initial state
  ROS_INFO("State prior to optimization");
  cal_job_->show();
  
  // Run observations and subsequent optimization
  ROS_INFO("RUNNING");
  if (cal_job_->run())
    {
      ROS_INFO_STREAM("Calibration job observations and optimization complete");
      calibrated_=true;
    }
  else
    {
      ROS_INFO_STREAM("Calibration job failed");
      return(false);
    }
  
  // Show Results
  cal_job_->show();
  
  // Store Results
  if (!cal_job_->store())
    {
      ROS_INFO_STREAM(" Trouble storing calibration job optimization results ");
    }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_service_node");
  ros::NodeHandle nh;
  CalibrationServiceNode cal_service_node(nh);

  ros::ServiceServer service=nh.advertiseService("calibration_service", &CalibrationServiceNode::callback, &cal_service_node);

  ros::spin();
    
}


