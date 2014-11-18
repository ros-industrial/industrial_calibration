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

#include <industrial_extrinsic_cal/calibration_job_definition.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <industrial_extrinsic_cal/manual_triggerAction.h>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/ros_triggers.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using ceres::CostFunction;

namespace industrial_extrinsic_cal
{


  bool CalibrationJob::load()
  {
    if(CalibrationJob::loadCamera())
      {
	ROS_INFO_STREAM("Successfully read in cameras ");
      }
    else
      {
	ROS_ERROR_STREAM("Camera file parsing failed");
	return false;
      }
    if(CalibrationJob::loadTarget())
      {
	ROS_INFO_STREAM("Successfully read in targets");
      }
    else
      {
	ROS_ERROR_STREAM("Target file parsing failed");
	return false;
      }
    if(CalibrationJob::loadCalJob())
      {
	ROS_INFO_STREAM("Successfully read in CalJob");
      }
    else
      {
	ROS_ERROR_STREAM("Calibration Job file parsing failed");
	return false;
      }

    return (true);
  } // end load()

  bool CalibrationJob::loadCamera()
  {
    string temp_name, temp_topic, camera_optical_frame, camera_housing_frame, camera_mounting_frame, parent_frame;
    CameraParameters temp_parameters;
    P_BLOCK extrinsics;

    unsigned int scene_id;
    std::string trigger_name;
    std::string trig_param;
    std::string trig_action_server;
    std::string trig_action_msg;
    std::string transform_interface;
    try
      {
	//	YAML::Parser camera_parser(camera_input_file);
	//	YAML::Node camera_doc;
	//	camera_parser.GetNextDocument(camera_doc);

	YAML::Node camera_doc = YAML::LoadFile(camera_def_file_name_.c_str());
	YAML::Node camera_parameters = camera_doc["static_cameras"];

	ROS_DEBUG_STREAM("Found " << camera_parameters.size() << " static cameras ");
	for (unsigned int i = 0; i < camera_parameters.size(); i++)// for each static camera
	  {
	    YAML::Node this_camera = camera_parameters[i];
	    temp_name                      = this_camera["camera_name"].as<std::string>();
	    trigger_name                   = this_camera["trigger"].as<std::string>();
	    temp_topic                     = this_camera["image_topic"].as<std::string>();
	    camera_optical_frame           = this_camera["camera_optical_frame"].as<std::string>();
	    transform_interface            = this_camera["transform_interface"].as<std::string>();
	    temp_parameters.angle_axis[0]  = this_camera["angle_axis_ax"].as<double>();
	    temp_parameters.angle_axis[1]  = this_camera["angle_axis_ay"].as<double>();
	    temp_parameters.angle_axis[2]  = this_camera["angle_axis_az"].as<double>();
	    temp_parameters.position[0]    = this_camera["position_x"].as<double>();
	    temp_parameters.position[1]    = this_camera["position_y"].as<double>();
	    temp_parameters.position[2]    = this_camera["position_z"].as<double>();
	    temp_parameters.focal_length_x = this_camera["focal_length_x"].as<double>();
	    temp_parameters.focal_length_y = this_camera["focal_length_y"].as<double>();
	    temp_parameters.center_x       = this_camera["center_x"].as<double>();
	    temp_parameters.center_y       = this_camera["center_y"].as<double>();
	    temp_parameters.distortion_k1  = this_camera["distortion_k1"].as<double>();
	    temp_parameters.distortion_k2  = this_camera["distortion_k2"].as<double>();
	    temp_parameters.distortion_k3  = this_camera["distortion_k3"].as<double>();
	    temp_parameters.distortion_p1  = this_camera["distortion_p1"].as<double>();
	    temp_parameters.distortion_p2  = this_camera["distortion_p2"].as<double>();
	    Pose6d pose(temp_parameters.position[0],temp_parameters.position[1],temp_parameters.position[2],
			temp_parameters.angle_axis[0],temp_parameters.angle_axis[1],temp_parameters.angle_axis[2]);
	    // create a shared camera and a shared transform interface
	    shared_ptr<Camera> temp_camera = make_shared<Camera>(temp_name, temp_parameters, false);
	    shared_ptr<TransformInterface> temp_ti;
    
	    // handle all the different trigger cases
	    if(trigger_name == std::string("NO_WAIT_TRIGGER")){
	      temp_camera->trigger_ = make_shared<NoWaitTrigger>();
	    }
	    else if(trigger_name == std::string("ROS_PARAM_TRIGGER")){
	      trig_param = this_camera["trig_param"].as<std::string>();
	      temp_camera->trigger_ = make_shared<ROSParamTrigger>(trig_param);
	    }
	    else if(trigger_name == std::string("ROS_ACTION_TRIGGER")){
	      trig_action_server = this_camera["trig_action_server"].as<std::string>();
	      trig_action_msg = this_camera["trig_action_msg"].as<std::string>();
	      temp_camera->trigger_ = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
	    }
	    else if(trigger_name == std::string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
	      trig_action_server = this_camera["trig_action_server"].as<std::string>();
	      std::vector<double>joint_values;
	      joint_values = this_camera["joint_values"].as< std::vector<double> >();
	      if(joint_values.size()<0){
		ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
	      }
	      temp_camera->trigger_ = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
	    }
	    else if(trigger_name == std::string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
	      trig_action_server = this_camera["trig_action_server"].as<std::string>();
	      geometry_msgs::Pose pose;
	      pose.position.x = this_camera["pose"][0].as<double>();
	      pose.position.y = this_camera["pose"][1].as<double>();
	      pose.position.z = this_camera["pose"][2].as<double>();
	      pose.orientation.x = this_camera["pose"][3].as<double>();
	      pose.orientation.y = this_camera["pose"][4].as<double>();
	      pose.orientation.z = this_camera["pose"][5].as<double>();
	      pose.orientation.w = this_camera["pose"][6].as<double>();
	      temp_camera->trigger_ = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
	    }
	    else{
	      ROS_ERROR("No scene trigger of type %s", trigger_name.c_str());
	    }
	    if(transform_interface == std::string("ros_lti")){ // this option makes no sense for a camera
	      temp_ti = make_shared<ROSListenerTransInterface>(camera_optical_frame);
	    }
	    else if(transform_interface == std::string("ros_bti")){ // this option makes no sense for a camera
	      temp_ti = make_shared<ROSBroadcastTransInterface>(camera_optical_frame, pose);
	    }
	    else if(transform_interface == std::string("ros_camera_lti")){ 
	      temp_ti = make_shared<ROSCameraListenerTransInterface>(camera_optical_frame);
	    }
	    else if(transform_interface == std::string("ros_camera_bti")){ 
	      temp_ti = make_shared<ROSCameraBroadcastTransInterface>(camera_optical_frame, pose);
	    }
	    else if(transform_interface == std::string("ros_camera_housing_lti")){ 
	      camera_housing_frame = this_camera["camera_housing_frame"].as<std::string>();
	      temp_ti = make_shared<ROSCameraHousingListenerTInterface>(camera_optical_frame,camera_housing_frame);
	    }
	    else if(transform_interface == std::string("ros_camera_housing_bti")){ 
	      camera_housing_frame = this_camera["camera_housing_frame"].as<std::string>();
	      camera_mounting_frame = this_camera["camera_mounting_frame"].as<std::string>(); 
	      temp_ti = make_shared<ROSCameraHousingBroadcastTInterface>(camera_optical_frame, camera_housing_frame, camera_mounting_frame, pose);
	    }
	    else if(transform_interface == std::string("ros_camera_housing_cti")){ 
	      camera_housing_frame  = this_camera["camera_housing_frame"].as<std::string>();
	      camera_mounting_frame = this_camera["camera_mounting_frame"].as<std::string>(); 
	      temp_ti = make_shared<ROSCameraHousingCalTInterface>(camera_optical_frame, 
								   camera_housing_frame,
								   camera_mounting_frame);
	    }
	    else if(transform_interface == std::string("ros_scti")){ 
	      camera_mounting_frame = this_camera["parent_frame"].as<std::string>(); 
	      temp_ti = make_shared<ROSSimpleCalTInterface>(camera_optical_frame,  camera_mounting_frame);
	    }
	    else if(transform_interface == std::string("default_ti")){
	      temp_ti = make_shared<DefaultTransformInterface>(pose);
	    }
	    else{
	      ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
	      temp_ti = make_shared<DefaultTransformInterface>(pose);
	    }
	    temp_camera->setTransformInterface(temp_ti);// install the transform interface 
	    temp_camera->camera_observer_ = make_shared<ROSCameraObserver>(temp_topic);
	    ceres_blocks_.addStaticCamera(temp_camera);
	  }// end for each static camera


	// read in all moving cameras
	camera_parameters = camera_doc["moving_cameras"];

	ROS_DEBUG_STREAM("Found " << camera_parameters.size() << " moving cameras ");
	for (unsigned int i = 0; i < camera_parameters.size(); i++)// for each moving camera
	  {
	    YAML::Node this_camera = camera_parameters[i];
	    
	    temp_name = this_camera["camera_name"].as<std::string>();
	    trigger_name = this_camera["trigger"].as<std::string>();
	    temp_topic = this_camera["image_topic"].as<std::string>();
	    camera_optical_frame = this_camera["camera_optical_frame"].as<std::string>();
	    transform_interface = this_camera["transform_interface"].as<std::string>();
	    temp_parameters.angle_axis[0]  = this_camera["angle_axis_ax"].as<double>();
	    temp_parameters.angle_axis[1]  = this_camera["angle_axis_ay"].as<double>();
	    temp_parameters.angle_axis[2]  = this_camera["angle_axis_az"].as<double>();
	    temp_parameters.position[0]    = this_camera["position_x"].as<double>();
	    temp_parameters.position[1]    = this_camera["position_y"].as<double>();
	    temp_parameters.position[2]    = this_camera["position_z"].as<double>();
	    temp_parameters.focal_length_x = this_camera["focal_length_x"].as<double>();
	    temp_parameters.focal_length_y = this_camera["focal_length_y"].as<double>();
	    temp_parameters.center_x       = this_camera["center_x"].as<double>();
	    temp_parameters.center_y       = this_camera["center_y"].as<double>();
	    temp_parameters.distortion_k1  = this_camera["distortion_k1"].as<double>();
	    temp_parameters.distortion_k2  = this_camera["distortion_k2"].as<double>();
	    temp_parameters.distortion_k3  = this_camera["distortion_k3"].as<double>();
	    temp_parameters.distortion_p1  = this_camera["distortion_p1"].as<double>();
	    temp_parameters.distortion_p2  = this_camera["distortion_p2"].as<double>();
		
	    Pose6d pose(temp_parameters.position[0],temp_parameters.position[1],temp_parameters.position[2],
			temp_parameters.angle_axis[0],temp_parameters.angle_axis[1],temp_parameters.angle_axis[2]);
	    // create a shared camera and a shared transform interface
	    shared_ptr<Camera> temp_camera = make_shared<Camera>(temp_name, temp_parameters, true);
	    shared_ptr<TransformInterface> temp_ti;

	    // handle all the different trigger cases
	    if(trigger_name == std::string("NO_WAIT_TRIGGER")){
	      temp_camera->trigger_ = make_shared<NoWaitTrigger>();
	    }
	    else if(trigger_name == std::string("ROS_PARAM_TRIGGER")){
	      trig_param = this_camera["trig_param"].as<std::string>();
	      temp_camera->trigger_ = make_shared<ROSParamTrigger>(trig_param);
	    }
	    else  if(trigger_name == std::string("ROS_ACTION_TRIGGER")){
	      trig_action_server = this_camera["trig_action_server"].as<std::string>();
	      trig_action_msg = this_camera["trig_action_message"].as<std::string>();
	      temp_camera->trigger_ = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
	    }
	    else if(trigger_name == std::string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
	      trig_action_server = this_camera["trig_action_server"].as<std::string>();
	      std::vector<double>joint_values;
	      joint_values = this_camera["joint_values"].as< std::vector<double> >();
	      if(joint_values.size()<0){
		ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
	      }
	      temp_camera->trigger_ = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
	    }
	    else if(trigger_name == std::string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
	      trig_action_server = this_camera["trig_action_server"].as<std::string>();
	      geometry_msgs::Pose pose;
	      pose.position.x = this_camera["pose"][0].as<double>();
	      pose.position.y = this_camera["pose"][1].as<double>();
	      pose.position.z = this_camera["pose"][2].as<double>();
	      pose.orientation.x = this_camera["pose"][3].as<double>();
	      pose.orientation.y = this_camera["pose"][4].as<double>();
	      pose.orientation.z = this_camera["pose"][5].as<double>();
	      pose.orientation.w = this_camera["pose"][6].as<double>();
	      temp_camera->trigger_ = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
	    }

	    // install camera's transform interface
	    if(transform_interface == std::string("ros_lti")){ // this option makes no sense for a camera
	      temp_ti = make_shared<ROSListenerTransInterface>(camera_optical_frame);
	    }
	    else if(transform_interface == std::string("ros_bti")){ // this option makes no sense for a camera
	      temp_ti = make_shared<ROSBroadcastTransInterface>(camera_optical_frame, pose);
	    }
	    else if(transform_interface == std::string("ros_camera_lti")){ 
	      temp_ti = make_shared<ROSCameraListenerTransInterface>(camera_optical_frame);
	    }
	    else if(transform_interface == std::string("ros_camera_bti")){ 
	      temp_ti = make_shared<ROSCameraBroadcastTransInterface>(camera_optical_frame, pose);
	    }
	    else if(transform_interface == std::string("ros_camera_housing_lti")){ 
	      camera_housing_frame = this_camera["camera_housing_frame"].as<std::string>();
	      temp_ti = make_shared<ROSCameraHousingListenerTInterface>(camera_optical_frame, camera_housing_frame);
	    }
	    else if(transform_interface == std::string("ros_camera_housing_bti")){ 
	      camera_housing_frame = this_camera["camera_housing_frame"].as<std::string>();
	      camera_mounting_frame = this_camera["camera_mounting_frame"].as<std::string>(); 
	      temp_ti = make_shared<ROSCameraHousingBroadcastTInterface>(camera_optical_frame, camera_housing_frame, camera_mounting_frame, pose);
	    }
	    else if(transform_interface == std::string("ros_camera_housing_cti")){ 
	      camera_housing_frame  = this_camera["camera_housing_frame"].as<std::string>();
	      camera_mounting_frame = this_camera["camera_mounting_frame"].as<std::string>(); 
	      temp_ti = make_shared<ROSCameraHousingCalTInterface>(camera_optical_frame, 
								   camera_housing_frame,
								   camera_mounting_frame);
	    }
	    else if(transform_interface == std::string("ros_scti")){ 
	      camera_mounting_frame = this_camera["parent_frame"].as<std::string>(); 
	      temp_ti = make_shared<ROSSimpleCalTInterface>(camera_optical_frame,  camera_mounting_frame);
	    }
	    else if(transform_interface == std::string("default_ti")){
	      temp_ti = make_shared<DefaultTransformInterface>(pose);
	    }
	    else{
	      ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
	      temp_ti = make_shared<DefaultTransformInterface>(pose);
	    }
	    temp_camera->setTransformInterface(temp_ti);// install the transform interface 
	    temp_camera->camera_observer_ = make_shared<ROSCameraObserver>(temp_topic);
	    ceres_blocks_.addMovingCamera(temp_camera, scene_id);
	  }// end for each moving camera
      } // end try
    catch (YAML::ParserException& e)
      {
	ROS_ERROR("load() Failed to read in cameras yaml file");
	ROS_ERROR_STREAM("Failed with exception "<< e.what());
	return (false);
      }
    return true;
  }
  
  bool CalibrationJob::loadTarget()
  {
    Target temp_target;
    std::string transform_interface;
    try
      {
	YAML::Node target_doc = YAML::LoadFile(target_def_file_name_.c_str());
	YAML::Node target_parameters = target_doc["static_targets"];
	ROS_DEBUG_STREAM("Found " << target_parameters.size() << " static targets ");
	for (unsigned int i = 0; i < target_parameters.size(); i++)// for each static target
	  {
	    YAML::Node this_target = target_parameters[i];
	    // create shared target and transform interface
	    shared_ptr<Target> temp_target = make_shared<Target>();
	    shared_ptr<TransformInterface> temp_ti;

	    temp_target->is_moving_ = false;
	    temp_target->target_name_ = this_target["target_name"].as<std::string>();
	    temp_target->target_frame_ = this_target["target_frame"].as<std::string>();
	    temp_target->target_type_ = this_target["target_type"].as<unsigned int>();
	    switch (temp_target->target_type_)
	      {
	      case pattern_options::Chessboard:
		temp_target->checker_board_parameters_.pattern_rows = this_target["target_rows"].as<int>();
		temp_target->checker_board_parameters_.pattern_cols = this_target["target_cols"].as<int>();
		ROS_DEBUG_STREAM("TargetRows: "<<temp_target->checker_board_parameters_.pattern_rows);
		break;
	      case pattern_options::CircleGrid:
		temp_target->circle_grid_parameters_.pattern_rows = this_target["target_rows"].as<int>();
		temp_target->circle_grid_parameters_.pattern_cols = this_target["target_cols"].as<int>();
		temp_target->circle_grid_parameters_.circle_diameter = this_target["circle_dia"].as<double>();
		temp_target->circle_grid_parameters_.is_symmetric=true;
		ROS_DEBUG_STREAM("TargetRows: "<<temp_target->circle_grid_parameters_.pattern_rows);
		break;
	      default:
		ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard or CircleGrid)");
		return false;
		break;
	      } // end of target type
	    temp_target->pose_.ax = this_target["angle_axis_ax"].as<double>();
	    temp_target->pose_.ay = this_target["angle_axis_ay"].as<double>();
	    temp_target->pose_.az = this_target["angle_axis_az"].as<double>();
	    temp_target->pose_.x = this_target["position_x"].as<double>();
	    temp_target->pose_.y = this_target["position_y"].as<double>();
	    temp_target->pose_.z = this_target["position_z"].as<double>();
	    transform_interface = this_target["transform_interface"].as<std::string>();
	    
	    // install target's transform interface
	    if(transform_interface == std::string("ros_lti")){ 
	      temp_ti = make_shared<ROSListenerTransInterface>(temp_target->target_frame_);
	    }
	    else if(transform_interface == std::string("ros_bti")){ 
	      temp_ti = make_shared<ROSBroadcastTransInterface>(temp_target->target_frame_, temp_target->pose_);
	    }
	    else if(transform_interface == std::string("ros_scti")){ 
	      std::string parent_frame;
	      parent_frame = this_target["parent_frame"].as<std::string>();
	      temp_ti = make_shared<ROSSimpleCalTInterface>(temp_target->target_frame_,  parent_frame);
	    }
	    
	    else if(transform_interface == std::string("default_ti")){
	      temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
	    }
	    else{
	      ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
	      temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
	    }
	    temp_target->setTransformInterface(temp_ti);// install the transform interface 
	    temp_target->num_points_ = this_target["num_points"].as<int>();
	    const YAML::Node points_node = this_target["points"];
	    ROS_DEBUG_STREAM("FoundPoints: " << points_node.size());
	    for (int j = 0; j < points_node.size(); j++)
	      {
		const YAML::Node this_point = points_node[j]["pnt"];
		Point3d temp_pnt3d;
		temp_pnt3d.x = this_point[0].as<double>();
		temp_pnt3d.y = this_point[1].as<double>();
		temp_pnt3d.z = this_point[2].as<double>();
		temp_target->pts_.push_back(temp_pnt3d);
	      }
	    if(temp_target->is_moving_ == true){
	      ROS_ERROR("Static Target set to moving????");
	    }
	    ceres_blocks_.addStaticTarget(temp_target);
	  } // end for each static target
    
    
	// read in all moving targets
	unsigned int scene_id;
	target_parameters = target_doc["moving_targets"];
	ROS_DEBUG_STREAM("Found " << target_parameters.size() << " moving targets ");
	for (unsigned int i = 0; i < target_parameters.size(); i++)// for each static target
	  {
	    YAML::Node this_target = target_parameters[i];
	    // create shared target and transform interface
	    shared_ptr<Target> temp_target = make_shared<Target>();
	    shared_ptr<TransformInterface> temp_ti;

	    temp_target->is_moving_ = true;
	    temp_target->target_name_ = this_target["target_name"].as<std::string>();
	    temp_target->target_frame_ = this_target["target_frame"].as<std::string>();
	    temp_target->target_type_ = this_target["target_type"].as<unsigned int>();
	    transform_interface = this_target["transform_interface"].as<std::string>();

	    // install target's transform interface
	    if(transform_interface == std::string("ros_lti")){ 
	      temp_ti = make_shared<ROSListenerTransInterface>(temp_target->target_frame_);
	    }
	    else if(transform_interface == std::string("ros_bti")){ 
	      temp_ti = make_shared<ROSBroadcastTransInterface>(temp_target->target_frame_, temp_target->pose_);
	    }
	    else if(transform_interface == std::string("default_ti")){
	      temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
	    }
	    else{
	      ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
	      temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
	    }
	    temp_target->setTransformInterface(temp_ti);// install the transform interface 
	    
	    // set parameters by the target's type
	    temp_target->target_type_ = this_target["target_type"].as<unsigned int>();

	    switch (temp_target->target_type_)
	      {
	      case pattern_options::Chessboard:
		temp_target->checker_board_parameters_.pattern_rows = this_target["target_rows"].as<int>();
		temp_target->checker_board_parameters_.pattern_cols = this_target["target_cols"].as<int>();
		ROS_INFO_STREAM("TargetRows: "<<temp_target->checker_board_parameters_.pattern_rows);
		break;
	      case pattern_options::CircleGrid:
		temp_target->circle_grid_parameters_.pattern_rows    = this_target["target_rows"].as<int>();
		temp_target->circle_grid_parameters_.pattern_cols    = this_target["target_cols"].as<int>();
		temp_target->circle_grid_parameters_.circle_diameter = this_target["circle_dia"].as<int>();
		temp_target->circle_grid_parameters_.is_symmetric=true;
		break;
	      default:
		ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard or CircleGrid)");
		return false;
		break;
	      }
	    temp_target->pose_.ax = this_target["angle_axis_ax"].as<double>();
	    temp_target->pose_.ay = this_target["angle_axis_ay"].as<double>();
	    temp_target->pose_.az = this_target["angle_axis_az"].as<double>();
	    temp_target->pose_.x = this_target["position_x"].as<double>();
	    temp_target->pose_.y = this_target["position_y"].as<double>();
	    temp_target->pose_.z = this_target["position_z"].as<double>();
	    transform_interface  = this_target["transform_interface"].as<std::string>();
	    if(transform_interface == std::string("ros_lti")){
	      temp_ti = make_shared<ROSListenerTransInterface>(temp_target->target_frame_);
	    }
	    if(transform_interface == std::string("ros_bti")){
	      temp_ti = make_shared<ROSBroadcastTransInterface>(temp_target->target_frame_,temp_target->pose_);
	    }
	    else if(transform_interface == std::string("default_ti")){
	      temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
	    }
	    else{
	      ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
	      temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
	    }
	    temp_target->setTransformInterface(temp_ti);// install the transform interface 

	    scene_id = this_target["scene_id"].as<unsigned int>();
	    temp_target->num_points_ = this_target["num_points"].as<int>();
	    const YAML::Node points_node = this_target["points"];
	    ROS_DEBUG_STREAM("FoundPoints: " << points_node.size());
	    for (int j = 0; j < points_node.size(); j++)
	      {
		const YAML::Node this_point = points_node[j]["pnt"];
		Point3d temp_pnt3d;
		temp_pnt3d.x = this_point[0].as<double>();
		temp_pnt3d.y = this_point[1].as<double>();
		temp_pnt3d.z = this_point[2].as<double>();
		temp_target->pts_.push_back(temp_pnt3d);
	      }
	    ceres_blocks_.addMovingTarget(temp_target, scene_id);
	  }// end for each moving target
      } // end try
    catch (YAML::ParserException& e)
      {
	ROS_ERROR("load() Failed to read in target yaml file");
	ROS_ERROR_STREAM("Failed with exception "<< e.what());
	return (false);
      }
    return true;

  }

  bool CalibrationJob::loadCalJob()
  {

    std::string opt_params;
    int scene_id_num;
    std::string trigger_name;
    std::string trig_param;
    std::string trig_action_server;
    std::string trig_action_msg;
    std::string camera_name;
    std::string target_name;
    std::string cost_type_string;
    Cost_function cost_type;
    std::string reference_frame;
    shared_ptr<Camera> temp_cam = make_shared<Camera>();
    shared_ptr<Target> temp_targ = make_shared<Target>();
    Roi temp_roi;

    try
      {
	YAML::Node caljob_doc = YAML::LoadFile(caljob_def_file_name_.c_str());
	reference_frame = caljob_doc["reference_frame"].as<std::string>();
	ceres_blocks_.setReferenceFrame(reference_frame);
	opt_params = caljob_doc["optimization_parameters"].as<std::string>();

	// read in all scenes
	YAML::Node caljob_scenes = caljob_doc["scenes"];
	int num_scenes = caljob_scenes.size();
	ROS_DEBUG_STREAM("Found " << num_scenes << " scenes");
	scene_list_.resize(num_scenes );
	for(int i=0; i<num_scenes; i++){// for each scene
	  YAML::Node this_scene = caljob_scenes[i];
	  scene_id_num = this_scene["scene_id"].as<int>();
	  trigger_name = this_scene["trigger"].as<std::string>();
	  string ros_bool_param;
	  string message;
	  string server_name;
	  boost::shared_ptr<Trigger> temp_trigger;
	  
	  // handle all the different trigger cases
	  if(trigger_name == std::string("NO_WAIT_TRIGGER")){
	    temp_trigger = make_shared<NoWaitTrigger>();
	  }
	  else if(trigger_name == std::string("ROS_PARAM_TRIGGER")){
	    trig_param = this_scene["trig_param"].as<std::string>();
	    temp_trigger = make_shared<ROSParamTrigger>(trig_param);
	  }
	  else if(trigger_name == std::string("ROS_ACTION_TRIGGER")){
	    trig_action_server = this_scene["trig_action_server"].as<std::string>();
	    trig_action_msg = this_scene["trig_action_msg"].as<std::string>();
	    temp_trigger = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
	  }
	  else if(trigger_name == std::string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
	    trig_action_server = this_scene["trig_action_server"].as<std::string>();
	    std::vector<double>joint_values;
	    joint_values = this_scene["joint_values"].as<std::vector<double> >();
	    if(joint_values.size()<1){
	      ROS_ERROR("Couldn't read  joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
	    }
	    temp_trigger = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
	  }
	  else if(trigger_name == std::string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
	    trig_action_server = this_scene["trig_action_server"].as<std::string>();
	    geometry_msgs::Pose pose;
	    pose.position.x = this_scene["pose"][0].as<double>();
	    pose.position.y = this_scene["pose"][1].as<double>();
	    pose.position.z = this_scene["pose"][2].as<double>();
	    pose.orientation.x = this_scene["pose"][3].as<double>();
	    pose.orientation.y = this_scene["pose"][4].as<double>();
	    pose.orientation.z = this_scene["pose"][5].as<double>();
	    pose.orientation.w = this_scene["pose"][6].as<double>();
	    temp_trigger = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
	  }
	  
	  scene_list_.at(i).setTrigger(temp_trigger);
	  scene_list_.at(i).setSceneId(scene_id_num);
	  
	  YAML::Node observations = this_scene["observations"];
	  ROS_DEBUG_STREAM("Found "<<observations.size() <<" observations within scene "<<i);
	  for (unsigned int j = 0; j < observations.size(); j++)// for each observation
	    {
	      YAML::Node this_observation = observations[j];
	      camera_name = this_observation["camera"].as<std::string>();
	      temp_roi.x_min = this_observation["roi_x_min"].as<int>();
	      temp_roi.x_max = this_observation["roi_x_max"].as<int>();
	      temp_roi.y_min = this_observation["roi_y_min"].as<int>();
	      temp_roi.y_max = this_observation["roi_y_max"].as<int>();
	      target_name = this_observation["target"].as<std::string>();
	      cost_type_string = this_observation["cost_type"].as<std::string>();
	      cost_type = string2CostType(cost_type_string);
	      if((temp_cam = ceres_blocks_.getCameraByName(camera_name)) == NULL){
		ROS_ERROR("Couldn't find camea %s",camera_name.c_str());
	      }
	      if((temp_targ = ceres_blocks_.getTargetByName(target_name)) == NULL){;
		ROS_ERROR("Couldn't find target %s",target_name.c_str());
	      }
	      scene_list_.at(i).addCameraToScene(temp_cam);
	      cost_type = string2CostType(cost_type_string);
	      scene_list_.at(i).populateObsCmdList(temp_cam, temp_targ, temp_roi, cost_type);
	    }// end for each observation
	} // end for each scene
  } // end try
  catch (YAML::ParserException& e)
    {
      ROS_ERROR("load() Failed to read in caljob yaml file");
      ROS_ERROR_STREAM("Failed with exception "<< e.what());
      return (false);
      }
  return true;
}

  bool CalibrationJob::run()
  {
    ROS_INFO("Running observations");
    runObservations();
    ROS_INFO("Running optimization");
    bool optimization_ran_ok = runOptimization();
    if(optimization_ran_ok){
      pushTransforms(); // sends updated transforms to their intefaces
    }
    else{
      ROS_ERROR("Optimization failed");
    }
    return(optimization_ran_ok);
  }

  bool CalibrationJob::runObservations()
  {
    // the result of this function are twofold
    // First, it fills up observation_data_point_list_ with lists of observationspercamera
    // Second it adds parameter blocks to the ceres_blocks
    // extrinsics and intrinsics for each static camera
    // The whole target for once every static target (parameter blocks are in  Pose6d and an array of points)
    // The whole target once a scene for each moving target
    observation_data_point_list_.clear(); // clear previously recorded observations

    // For each scene
    BOOST_FOREACH(ObservationScene current_scene, scene_list_)
      {
	int scene_id = current_scene.get_id();
	ROS_DEBUG_STREAM("Processing Scene " << scene_id+1<<" of "<< scene_list_.size());
	ROS_INFO("Processing Scene  %d of %d",scene_id, (int) scene_list_.size());

	BOOST_FOREACH(shared_ptr<Camera> current_camera, current_scene.cameras_in_scene_)
	  {			// clear camera of existing observations
	    current_camera->camera_observer_->clearObservations(); // clear any recorded data
	    current_camera->camera_observer_->clearTargets(); // clear all targets
	    if(current_camera->isMoving()){
	      ROS_ERROR("Camera %s is moving in scene %d",current_camera->camera_name_.c_str(), scene_id);
	    }
	  }

	BOOST_FOREACH(ObservationCmd o_command, current_scene.observation_command_list_)
	  {	// add each target and roi each camera's list of observations
	    o_command.camera->camera_observer_->addTarget(o_command.target, o_command.roi, o_command.cost_type);
	  }
	
	current_scene.get_trigger()->waitForTrigger(); // this indicates scene is ready to capture

	pullTransforms(scene_id); // gets transforms of targets and cameras from their interfaces
	
	BOOST_FOREACH( shared_ptr<Camera> current_camera, current_scene.cameras_in_scene_)
	  {// trigger the cameras
	    P_BLOCK tmp;
	    current_camera->camera_observer_->triggerCamera();
	  }

	// collect results
	P_BLOCK intrinsics;
	P_BLOCK extrinsics;
	P_BLOCK target_pose;
	P_BLOCK pnt_pos;
	std::string camera_name;
	std::string target_name;
	int target_type;
	Cost_function cost_type;

	// for each camera in scene get a list of observations, and add camera parameters to ceres_blocks
	ObservationDataPointList listpercamera;
	BOOST_FOREACH( shared_ptr<Camera> camera, current_scene.cameras_in_scene_)
	  {
	    // wait until observation is done
	    while (!camera->camera_observer_->observationsDone()) ;

	    camera_name = camera->camera_name_;
	    if (camera->isMoving())
	      {
		// next line does nothing if camera already exist in blocks
		ceres_blocks_.addMovingCamera(camera, scene_id);
		pullTransforms(scene_id); // gets transforms of targets and cameras from their interfaces
		intrinsics = ceres_blocks_.getMovingCameraParameterBlockIntrinsics(camera_name);
		extrinsics = ceres_blocks_.getMovingCameraParameterBlockExtrinsics(camera_name, scene_id);
	      }
	    else
	      {
		// next line does nothing if camera already exist in blocks
		ceres_blocks_.addStaticCamera(camera);
		intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(camera_name);
		extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(camera_name);
	      }

	    // Get the observations from this camera whose P_BLOCKs are intrinsics and extrinsics
	    CameraObservations camera_observations;
	    int number_returned;
	    number_returned = camera->getObservations(camera_observations);

	    ROS_DEBUG_STREAM("Processing " << camera_observations.size() << " Observations");
	    ROS_INFO("Processing %d Observations ", (int) camera_observations.size());
	    BOOST_FOREACH(Observation observation, camera_observations)
	      {
		target_name = observation.target->target_name_;
		target_type = observation.target->target_type_;
		cost_type = observation.cost_type;
		double circle_dia=0.0;
		if(target_type == pattern_options::CircleGrid){
		  circle_dia = observation.target->circle_grid_parameters_.circle_diameter;
		}
		int pnt_id = observation.point_id;
		double observation_x = observation.image_loc_x;
		double observation_y = observation.image_loc_y;
		if (observation.target->is_moving_)
		  {
		    ceres_blocks_.addMovingTarget(observation.target, scene_id);
		    target_pose = ceres_blocks_.getMovingTargetPoseParameterBlock(target_name, scene_id);
		    pnt_pos = ceres_blocks_.getMovingTargetPointParameterBlock(target_name, pnt_id);
		  }
		else
		  {
		    ceres_blocks_.addStaticTarget(observation.target); // if exist, does nothing
		    target_pose = ceres_blocks_.getStaticTargetPoseParameterBlock(target_name);
		    pnt_pos = ceres_blocks_.getStaticTargetPointParameterBlock(target_name, pnt_id);
		  }
		ObservationDataPoint temp_ODP(camera_name, target_name, target_type,
					      scene_id, intrinsics, extrinsics, pnt_id, target_pose,
					      pnt_pos, observation_x, observation_y, 
					      cost_type, observation.intermediate_frame,
					      circle_dia);
		listpercamera.addObservationPoint(temp_ODP);
	      }//end for each observed point
	  }//end for each camera
	observation_data_point_list_.push_back(listpercamera);
      } //end for each scene
    return true;
  }

  bool CalibrationJob::runOptimization()
  {
    int total_observations =0;
    for(int i=0;i<observation_data_point_list_.size();i++){
      total_observations += observation_data_point_list_[i].items_.size();
      std::stringstream observations_ss;
      for (int pntIdx = 0; pntIdx < observation_data_point_list_[i].items_.size(); pntIdx++) {
        const ObservationDataPoint& odp = observation_data_point_list_[i].items_[pntIdx];
        ROS_INFO("%d: id=%d pos=[%f, %f, %f] img=[%f, %f]", pntIdx,
            odp.point_id_,
            odp.point_position_[0], odp.point_position_[1], odp.point_position_[2],
            odp.image_x_, odp.image_y_);
        observations_ss << "[" << odp.image_x_ << ", " << odp.image_y_ << "],";
      }
      ROS_INFO("project_points2d: %s", observations_ss.str().c_str());
    }
    if(total_observations == 0){ // TODO really need more than number of parameters being computed
      ROS_ERROR("Too few observations: %d",total_observations);
      return(false);
    }
    
    ceres_blocks_.displayMovingCameras();

    // take all the data collected and create a Ceres optimization problem and run it
    ROS_INFO("Running Optimization with %d scenes",(int)scene_list_.size());
    ROS_DEBUG_STREAM("Optimizing "<<scene_list_.size()<<" scenes");
    BOOST_FOREACH(ObservationScene current_scene, scene_list_)
      {

	int scene_id = current_scene.get_id();
	BOOST_FOREACH(shared_ptr<Camera> camera, current_scene.cameras_in_scene_)
	  {
	    ROS_DEBUG_STREAM("Current observation data point list size: "<<observation_data_point_list_.at(scene_id).items_.size());
	    // take all the data collected and create a Ceres optimization problem and run it
	    P_BLOCK extrinsics;
	    P_BLOCK intrinsics;
	    P_BLOCK target_pose_params;
	    P_BLOCK point_position;
	    BOOST_FOREACH(ObservationDataPoint ODP, observation_data_point_list_.at(scene_id).items_)
	      {
		// create cost function
		// there are several options
		// 1. the complete reprojection error cost function "Create(obs_x,obs_y)"
		//    this cost function has the following parameters:
		//      a. camera intrinsics
		//      b. camera extrinsics
		//      c. target pose
		//      d. point location in target frame
		// 2. the same as 1, but without d  "Create(obs_x,obs_y,t_pnt_x, t_pnt_y, t_pnt_z)
		// 3. the same as 1, but without a  "Create(obs_x,obs_y,fx,fy,cx,cy,cz)"
		//    Note that this one assumes we are using rectified images to compute the observations
		// 4. the same as 3, point location fixed too "Create(obs_x,obs_y,fx,fy,cx,cy,cz,t_x,t_y,t_z)"
		//        implemented in TargetCameraReprjErrorNoDistortion
		// 5. the same as 4, but with target in known location
		//    "Create(obs_x,obs_y,fx,fy,cx,cy,cz,t_x,t_y,t_z,p_tx,p_ty,p_tz,p_ax,p_ay,p_az)"
		// pull out the constants from the observation point data
		intrinsics = ODP.camera_intrinsics_;
		double focal_length_x = ODP.camera_intrinsics_[0]; // TODO, make this not so ugly
		double focal_length_y = ODP.camera_intrinsics_[1];
		double center_x   = ODP.camera_intrinsics_[2];
		double center_y   = ODP.camera_intrinsics_[3];
		double image_x        = ODP.image_x_;
		double image_y        = ODP.image_y_;
		Point3d point;
		Pose6d camera_mounting_pose = ODP.intermediate_frame_; // identity except when camera mounted on robot
		point.x = ODP.point_position_[0];// location of point within target frame
		point.y = ODP.point_position_[1];
		point.z = ODP.point_position_[2];
		unsigned int target_type    = ODP.target_type_;
		double circle_dia = ODP.circle_dia_; // sometimes this is not needed
	      
		// pull out pointers to the parameter blocks in the observation point data
		extrinsics        = ODP.camera_extrinsics_;
		target_pose_params     = ODP.target_pose_;
		Pose6d target_pose;
		target_pose.setAngleAxis(target_pose_params[0],target_pose_params[1], target_pose_params[2]);
		target_pose.setOrigin(target_pose_params[3],target_pose_params[4], target_pose_params[5]);
		point_position = ODP.point_position_;
		bool point_zero=false;
		/*
		if(point.x == 0.0 && point.y == 0.0 && point.z == 0.0){
		  point_zero=true;
		  ROS_ERROR("Observing Target Origin");
		  showPose(target_pose_params, "target");
		  showPose(extrinsics,"extrinsics");
		  showPose((P_BLOCK) &camera_mounting_pose.pb_pose[0], "camera_mounting_pose");
		}
        */
		
		switch( ODP.cost_type_ ){
		case cost_functions::CameraReprjErrorWithDistortion:
		  {
		    CostFunction* cost_function =
		      CameraReprjErrorWithDistortion::Create(image_x, image_y);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
		  }
		  break;
		case cost_functions::CameraReprjErrorWithDistortionPK:
		  {
		    CostFunction* cost_function =
		      CameraReprjErrorWithDistortionPK::Create(image_x, image_y, 
							       point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, intrinsics);
		  }
		  break;
		case cost_functions::CameraReprjError:
		  {
		    CostFunction* cost_function =
		      CameraReprjError::Create(image_x, image_y, 
					       focal_length_x, focal_length_y,
					       center_x, center_y);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, point.pb);
		  }
		  break;
		case cost_functions::CameraReprjErrorPK:
		  {
		    CostFunction* cost_function =
		      CameraReprjErrorPK::Create(image_x, image_y, 
						 focal_length_x, focal_length_y,
						 center_x, center_y,
						 point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics);
		  }
		  break;
		case cost_functions::TargetCameraReprjError:
		  {
		    CostFunction* cost_function =
		      TargetCameraReprjError::Create(image_x, image_y, 
						     focal_length_x, focal_length_y,
						     center_x, center_y);

		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::TargetCameraReprjErrorPK:
		  {
		    CostFunction* cost_function = 
		      TargetCameraReprjErrorPK::Create(image_x, image_y,
						       focal_length_x,
						       focal_length_y,
						       center_x,
						       center_y,
						       point);
		    // add it as a residual using parameter blocks
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
		  }
		  break;
		case cost_functions::LinkTargetCameraReprjError:
		  {
		    CostFunction* cost_function =
		      LinkTargetCameraReprjError::Create(image_x, image_y, 
							 focal_length_x,
							 focal_length_y,
							 center_x,
							 center_y,
							 camera_mounting_pose);
		      problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::LinkTargetCameraReprjErrorPK:
		  {
		    CostFunction* cost_function =
		      LinkTargetCameraReprjErrorPK::Create(image_x, image_y, 
							   focal_length_x,
							   focal_length_y,
							   center_x,
							   center_y,
							   camera_mounting_pose,
							   point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
		  }
		  break;
		case cost_functions::LinkCameraTargetReprjError:
		  {
		    CostFunction* cost_function =
		      LinkCameraTargetReprjError::Create(image_x, image_y, 
							 focal_length_x,
							 focal_length_y,
							 center_x,
							 center_y,
							 camera_mounting_pose);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::LinkCameraTargetReprjErrorPK:
		    {
		      CostFunction* cost_function =
			LinkCameraTargetReprjErrorPK::Create(image_x, image_y, 
							     focal_length_x,
							     focal_length_y,
							     center_x,
							     center_y,
							     camera_mounting_pose,
							     point);
		      
		      problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
		    }
		    break;
		case cost_functions::CircleCameraReprjErrorWithDistortion:
		  {
		    CostFunction* cost_function =
		      CircleCameraReprjErrorWithDistortion::Create(image_x, image_y, circle_dia);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
		  }
		  break;
		case cost_functions::CircleCameraReprjErrorWithDistortionPK:
		  {
		    CostFunction* cost_function =
		      CircleCameraReprjErrorWithDistortionPK::Create(image_x, image_y,
								     circle_dia,
								     point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
		  }
		  break;
		case cost_functions::CircleCameraReprjError:
		  {
		    CostFunction* cost_function =
		      CircleCameraReprjError::Create(image_x, image_y, 
						     circle_dia,
						     focal_length_x,
						     focal_length_y,
						     center_x,
						     center_y);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, point.pb);
		  }
		  break;
		case cost_functions::CircleCameraReprjErrorPK:
		  {
		    CostFunction* cost_function =
		      CircleCameraReprjErrorPK::Create(image_x, image_y, 
						       circle_dia,
						       focal_length_x,
						       focal_length_y,
						       center_x,
						       center_y,
						       point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics);
		  }
		  break;
		case cost_functions::CircleTargetCameraReprjErrorWithDistortion:
		  {
		    CostFunction* cost_function =
		      CircleTargetCameraReprjErrorWithDistortion::Create(image_x, image_y,
									 circle_dia);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::CircleTargetCameraReprjErrorWithDistortionPK:
		  {
		    CostFunction* cost_function =
		      CircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, 
									   circle_dia,
									   point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, target_pose_params);
		  }
		  break;
		case cost_functions::CircleTargetCameraReprjError:
		  {
		    CostFunction* cost_function =
		      CircleTargetCameraReprjError::Create(image_x, image_y, 
							   circle_dia,
							   focal_length_x,
							   focal_length_y,
							   center_x,
							   center_y);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::CircleTargetCameraReprjErrorPK:
		  {
		    CostFunction* cost_function = 
		      CircleTargetCameraReprjErrorPK::Create(image_x,  image_y,
							     circle_dia,
							     focal_length_x,
							     focal_length_y,
							     center_x,
							     center_y,
							     point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
		  }
		  break;
		case cost_functions::LinkCircleTargetCameraReprjError:
		  {
		    CostFunction* cost_function =
		      LinkCircleTargetCameraReprjError::Create(image_x, image_y, 
							       circle_dia,
							       focal_length_x,
							       focal_length_y,
							       center_x,
							       center_y,
							       camera_mounting_pose);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::LinkCircleTargetCameraReprjErrorPK:
		  {
		    CostFunction* cost_function =
		      LinkCircleTargetCameraReprjErrorPK::Create(image_x, image_y, 
								 circle_dia,
								 focal_length_x,
								 focal_length_y,
								 center_x,
								 center_y,
								 camera_mounting_pose,
								 point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
		  }
		  break;
		case cost_functions::LinkCameraCircleTargetReprjError:
		  {
		    CostFunction* cost_function =
		      LinkCameraCircleTargetReprjError::Create(image_x, image_y, 
							       circle_dia,
							       focal_length_x,
							       focal_length_y,
							       center_x,
							       center_y,
							       camera_mounting_pose);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
		  }
		  break;
		case cost_functions::LinkCameraCircleTargetReprjErrorPK:
		  {
		    CostFunction* cost_function =
		      LinkCameraCircleTargetReprjErrorPK::Create(image_x, image_y, 
								 circle_dia,
								 focal_length_x,
								 focal_length_y,
								 center_x,
								 center_y,
								 camera_mounting_pose,
								 point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
		    if(point_zero){
		      double residual[2];
		      double *params[2];
		      params[0] = &extrinsics[0];
		      params[1] = &target_pose_params[0];
		      cost_function->Evaluate(params, residual, NULL);
		      ROS_INFO("Initial residual %6.3lf %6.3lf ix,iy = %6.3lf %6.3lf px,py = %6.3lf %6.3lf", residual[0], residual[1],image_x, image_y, residual[0]+image_x, residual[0]+image_y);
		      point_zero=false;
		      LinkCameraCircleTargetReprjErrorPK testIt(image_x, image_y, 
								circle_dia,
								focal_length_x,
								focal_length_y,
								center_x,
								center_y,
								camera_mounting_pose,
								point);
		      testIt.test_residual(extrinsics, target_pose_params, residual);

		    }
		  }
		  break;
		case cost_functions::FixedCircleTargetCameraReprjErrorPK:
		  {
		    CostFunction* cost_function =
		      FixedCircleTargetCameraReprjErrorPK::Create(image_x, image_y, 
								  circle_dia,
								  focal_length_x,
								  focal_length_y,
								  center_x,
								  center_y,
								  target_pose,
								  camera_mounting_pose,
								  point);
		    problem_.AddResidualBlock(cost_function, NULL , extrinsics);
		    if(point_zero){
		      double residual[2];
		      double *params[2];
		      params[0] = &extrinsics[0];
		      cost_function->Evaluate(params, residual, NULL);
		      ROS_ERROR("Initial residual %6.3lf %6.3lf ix,iy = %6.3lf %6.3lf px,py = %6.3lf %6.3lf", residual[0], residual[1],image_x, image_y, residual[0]+image_x, residual[0]+image_y);
		      point_zero=false;
		      FixedCircleTargetCameraReprjErrorPK testIt(image_x, image_y, 
								 circle_dia,
								 focal_length_x,
								 focal_length_y,
								 center_x,
								 center_y,
								 target_pose,
								 camera_mounting_pose,
								 point);
		      testIt.test_residual(extrinsics, residual);

		    }
		  }
		  break;
		default:
		  {
		    std::string cost_type_string = costType2String(ODP.cost_type_);
		    ROS_ERROR("No cost function of type %s", cost_type_string.c_str());
		  }
		  break;
		}// end of switch
	      }//for each observation
	  }//for each camera
      }//for each scene
  ROS_INFO("total observations: %d ",total_observations);
  
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem_, &summary);
  ROS_INFO("PROBLEM SOLVED");
  return true;
}//end runOptimization

  bool CalibrationJob::store()
  {
    std::string path = ros::package::getPath("industrial_extrinsic_cal");
    std::string file_path = "/launch/target_to_camera_optical_transform_publisher.launch";
    std::string filepath = path+file_path;

    bool rnt =  ceres_blocks_.writeAllStaticTransforms(filepath);
    bool rtn = true;
    return rtn;
  }

  void CalibrationJob::show()
  {
    ceres_blocks_.pullTransforms(-1); // since we don't know which scene for any moving objects, only pull static transforms
    ceres_blocks_.displayAllCamerasAndTargets();
  }
  void CalibrationJob::pullTransforms(int scene_id)
  {
    ceres_blocks_.pullTransforms( scene_id);
  }
  void CalibrationJob::pushTransforms()
  {
    ceres_blocks_.pushTransforms();
  }
}//end namespace industrial_extrinsic_cal
