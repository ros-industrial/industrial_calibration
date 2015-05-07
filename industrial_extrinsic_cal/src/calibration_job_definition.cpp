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
using industrial_extrinsic_cal::covariance_requests::CovarianceRequestType;

namespace industrial_extrinsic_cal
{
  
  /** @brief, a function used for debugging, and generating output for post processing */
#define WRITE_DEBUG true
#define OUTPUT_DEBUG_FILE "/home/nist/post_processing_data.txt"
  void writeObservationData(std::string file_name, std::vector<ObservationDataPointList> odl)
{
  FILE *fp=NULL;
  fp = fopen(file_name.c_str(), "w");
  if(fp == NULL){
    ROS_ERROR("Could not open %s", file_name.c_str());
    return;
  }
  
  int scene_id=-1;  
  ROS_ERROR("processing %d lists", (int) odl.size());
  for(int i=0; (int) i<odl.size(); i++){
    if(odl[i].items_.size()>0){
      scene_id = odl[i].items_[0].scene_id_;
      fprintf(fp, "scene_id = %d \n", scene_id);
      Pose6d t_pose;
      t_pose.ax = odl[i].items_[0].target_pose_[0];
      t_pose.ay = odl[i].items_[0].target_pose_[1];
      t_pose.az = odl[i].items_[0].target_pose_[2];
      t_pose.x  = odl[i].items_[0].target_pose_[3];
      t_pose.y  = odl[i].items_[0].target_pose_[4];
      t_pose.z  = odl[i].items_[0].target_pose_[5];
      tf::Matrix3x3 basis = t_pose.getBasis();
      fprintf(fp, "target_pose = [ %f %f %f %f;\n %f %f %f %f;\n %f %f %f %f;\n %f %f %f %f];\n",
	      basis[0][0],basis[0][1], basis[0][2],t_pose.x,
	      basis[1][0],basis[1][1], basis[1][2],t_pose.y,
	      basis[2][0],basis[2][1], basis[2][2],t_pose.z,
	      0.0, 0.0, 0.0, 1.0);
      fprintf(fp, "cameras = [ ");
      std::string camera_name("NULL");
      for(int j=0; (int) j<odl[i].items_.size(); j++){
	if(camera_name !=  odl[i].items_[j].camera_name_){
	  fprintf(fp,"%s ", odl[i].items_[j].camera_name_.c_str());
	}
	camera_name =  odl[i].items_[j].camera_name_;
      }
      fprintf(fp, "]\n");
    }
  }
  fclose(fp);
} 
  CovarianceRequestType intToCovRequest(int request)
  {
    switch (request){
    case 0:
      return covariance_requests::StaticCameraIntrinsicParams;
      break;
    case 1:
      return covariance_requests::StaticCameraExtrinsicParams;
      break;
    case 2:
      return covariance_requests::MovingCameraIntrinsicParams;
      break;
    case 3:
      return covariance_requests::MovingCameraExtrinsicParams;
      break;
    case 4:
      return covariance_requests::StaticTargetPoseParams;
      break;
    case 5:
      return covariance_requests::MovingTargetPoseParams;
      break;
    default:
      return covariance_requests::DefaultInvalid;
      break;
    }
  }// end of intToCovRequestType

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
    std::ifstream camera_input_file(camera_def_file_name_.c_str());
    if (camera_input_file.fail())
      {
	ROS_ERROR_STREAM(
			 "CalibrationJob::load(), couldn't open camera_input_file:    "
			 << camera_def_file_name_.c_str());
	return (false);
      }

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
	YAML::Parser camera_parser(camera_input_file);
	YAML::Node camera_doc;
	camera_parser.GetNextDocument(camera_doc);

	// read in all static cameras
	if (const YAML::Node *camera_parameters = camera_doc.FindValue("static_cameras"))
	  {
	    ROS_DEBUG_STREAM("Found "<<camera_parameters->size()<<" static cameras ");
	    for (unsigned int i = 0; i < camera_parameters->size(); i++)
	      {
		(*camera_parameters)[i]["camera_name"] >> temp_name;
		(*camera_parameters)[i]["trigger"] >> trigger_name;
		(*camera_parameters)[i]["image_topic"] >> temp_topic;
		(*camera_parameters)[i]["camera_optical_frame"] >> camera_optical_frame;
		(*camera_parameters)[i]["transform_interface"] >> transform_interface;
		(*camera_parameters)[i]["angle_axis_ax"] >> temp_parameters.angle_axis[0];
		(*camera_parameters)[i]["angle_axis_ay"] >> temp_parameters.angle_axis[1];
		(*camera_parameters)[i]["angle_axis_az"] >> temp_parameters.angle_axis[2];
		(*camera_parameters)[i]["position_x"] >> temp_parameters.position[0];
		(*camera_parameters)[i]["position_y"] >> temp_parameters.position[1];
		(*camera_parameters)[i]["position_z"] >> temp_parameters.position[2];
		(*camera_parameters)[i]["focal_length_x"] >> temp_parameters.focal_length_x;
		(*camera_parameters)[i]["focal_length_y"] >> temp_parameters.focal_length_y;
		(*camera_parameters)[i]["center_x"] >> temp_parameters.center_x;
		(*camera_parameters)[i]["center_y"] >> temp_parameters.center_y;
		(*camera_parameters)[i]["distortion_k1"] >> temp_parameters.distortion_k1;
		(*camera_parameters)[i]["distortion_k2"] >> temp_parameters.distortion_k2;
		(*camera_parameters)[i]["distortion_k3"] >> temp_parameters.distortion_k3;
		(*camera_parameters)[i]["distortion_p1"] >> temp_parameters.distortion_p1;
		(*camera_parameters)[i]["distortion_p2"] >> temp_parameters.distortion_p2;
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
		  (*camera_parameters)[i]["trig_param"] >> trig_param;
		  temp_camera->trigger_ = make_shared<ROSParamTrigger>(trig_param);
		}
		else if(trigger_name == std::string("ROS_ACTION_TRIGGER")){
		  (*camera_parameters)[i]["trig_action_server"] >> trig_action_server;
		  (*camera_parameters)[i]["trig_action_msg"] >> trig_action_msg;
		  temp_camera->trigger_ = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
		}
		else if(trigger_name == std::string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
		  (*camera_parameters)[i]["trig_action_server"] >> trig_action_server;
		  std::vector<double>joint_values;
		  (*camera_parameters)[i]["joint_values"] >> joint_values;
		  if(joint_values.size()<0){
		    ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
		  }
		  temp_camera->trigger_ = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
		}
		else if(trigger_name == std::string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
		  (*camera_parameters)[i]["trig_action_server"] >> trig_action_server;
		 Pose6d pose;
		  (*camera_parameters)[i]["pose"][0] >> pose.x;
		  (*camera_parameters)[i]["pose"][1] >> pose.y;
		  (*camera_parameters)[i]["pose"][2] >> pose.z;
		  double qx,qy,qz,qw;
		  (*camera_parameters)[i]["pose"][3] >> qx;
		  (*camera_parameters)[i]["pose"][4] >> qy;
		  (*camera_parameters)[i]["pose"][5] >> qz;
		  (*camera_parameters)[i]["pose"][6] >> qw;
		  pose.setQuaternion(qx, qy, qz, qw);
		  temp_camera->trigger_ = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
		}
		else{
		  ROS_ERROR("No scene trigger of type %s", trigger_name.c_str());
		}
		// parse the transform interface
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
		  (*camera_parameters)[i]["camera_housing_frame"] >> camera_housing_frame;
		  temp_ti = make_shared<ROSCameraHousingListenerTInterface>(camera_optical_frame,camera_housing_frame);
		}
		else if(transform_interface == std::string("ros_camera_housing_bti")){ 
		  (*camera_parameters)[i]["camera_housing_frame"] >> camera_housing_frame; // note, this is unused
		  temp_ti = make_shared<ROSCameraHousingBroadcastTInterface>(camera_optical_frame,  pose);
		}
		else if(transform_interface == std::string("ros_camera_housing_cti")){ 
		  (*camera_parameters)[i]["camera_housing_frame"] >> camera_housing_frame; 
		  (*camera_parameters)[i]["camera_mounting_frame"] >> camera_mounting_frame; 
		  temp_ti = make_shared<ROSCameraHousingCalTInterface>(camera_optical_frame, 
								       camera_housing_frame,
								       camera_mounting_frame);
		}
		else if(transform_interface == std::string("ros_scti")){ 
		  (*camera_parameters)[i]["parent_frame"] >> camera_mounting_frame; 
		  temp_ti = make_shared<ROSSimpleCalTInterface>(camera_optical_frame,  camera_mounting_frame);
		}
		else if(transform_interface == std::string("ros_camera_scti")){ 
		  (*camera_parameters)[i]["parent_frame"] >> camera_mounting_frame; 
		  temp_ti = make_shared<ROSSimpleCameraCalTInterface>(camera_optical_frame,  camera_mounting_frame);
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
		
	      }
	  }

	// read in all moving cameras
	if (const YAML::Node *camera_parameters = camera_doc.FindValue("moving_cameras"))
	  {
	    ROS_DEBUG_STREAM("Found "<<camera_parameters->size() << " moving cameras ");
	    for (unsigned int i = 0; i < camera_parameters->size(); i++)
	      {
		(*camera_parameters)[i]["camera_name"] >> temp_name;
		(*camera_parameters)[i]["trigger"] >> trigger_name;
		(*camera_parameters)[i]["image_topic"] >> temp_topic;
		(*camera_parameters)[i]["camera_optical_frame"] >> camera_optical_frame;
		(*camera_parameters)[i]["transform_interface"] >> transform_interface;
		(*camera_parameters)[i]["angle_axis_ax"] >> temp_parameters.angle_axis[0];
		(*camera_parameters)[i]["angle_axis_ay"] >> temp_parameters.angle_axis[1];
		(*camera_parameters)[i]["angle_axis_az"] >> temp_parameters.angle_axis[2];
		(*camera_parameters)[i]["position_x"] >> temp_parameters.position[0];
		(*camera_parameters)[i]["position_y"] >> temp_parameters.position[1];
		(*camera_parameters)[i]["position_z"] >> temp_parameters.position[2];
		(*camera_parameters)[i]["focal_length_x"] >> temp_parameters.focal_length_x;
		(*camera_parameters)[i]["focal_length_y"] >> temp_parameters.focal_length_y;
		(*camera_parameters)[i]["center_x"] >> temp_parameters.center_x;
		(*camera_parameters)[i]["center_y"] >> temp_parameters.center_y;
		(*camera_parameters)[i]["distortion_k1"] >> temp_parameters.distortion_k1;
		(*camera_parameters)[i]["distortion_k2"] >> temp_parameters.distortion_k2;
		(*camera_parameters)[i]["distortion_k3"] >> temp_parameters.distortion_k3;
		(*camera_parameters)[i]["distortion_p1"] >> temp_parameters.distortion_p1;
		(*camera_parameters)[i]["distortion_p2"] >> temp_parameters.distortion_p2;
		
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
		  (*camera_parameters)[i]["trig_param"] >> trig_param;
		  temp_camera->trigger_ = make_shared<ROSParamTrigger>(trig_param);
		}
		else  if(trigger_name == std::string("ROS_ACTION_TRIGGER")){
		  (*camera_parameters)[i]["trig_action_server"] >> trig_action_server;
		  (*camera_parameters)[i]["trig_action_message"] >> trig_action_msg;
		  temp_camera->trigger_ = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
		}
		else if(trigger_name == std::string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
		  (*camera_parameters)[i]["trig_action_server"] >> trig_action_server;
		  std::vector<double>joint_values;
		  (*camera_parameters)[i]["joint_values"] >> joint_values;
		  if(joint_values.size()<0){
		    ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
		  }
		  temp_camera->trigger_ = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
		}
		else if(trigger_name == std::string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
		  (*camera_parameters)[i]["trig_action_server"] >> trig_action_server;
		  Pose6d pose;
		  (*camera_parameters)[i]["pose"][0] >> pose.x;
		  (*camera_parameters)[i]["pose"][1] >> pose.y;
		  (*camera_parameters)[i]["pose"][2] >> pose.z;
		  double qx,qy,qz,qw;
		  (*camera_parameters)[i]["pose"][3] >> qx;
		  (*camera_parameters)[i]["pose"][4] >> qy;
		  (*camera_parameters)[i]["pose"][5] >> qz;
		  (*camera_parameters)[i]["pose"][6] >> qw;
		  pose.setQuaternion(qx, qy, qz, qw);
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
		  (*camera_parameters)[i]["camera_housing_frame"] >> camera_housing_frame; 
		  temp_ti = make_shared<ROSCameraHousingListenerTInterface>(camera_optical_frame, camera_housing_frame);
		}
		else if(transform_interface == std::string("ros_camera_housing_bti")){ 
		  (*camera_parameters)[i]["camera_housing_frame"] >> camera_housing_frame; // note, this is unused
		  temp_ti = make_shared<ROSCameraHousingBroadcastTInterface>(camera_optical_frame, pose);
		}
		else if(transform_interface == std::string("ros_camera_housing_cti")){ 
		  (*camera_parameters)[i]["camera_housing_frame"] >> camera_housing_frame; 
		  (*camera_parameters)[i]["camera_mounting_frame"] >> camera_mounting_frame; 
		  temp_ti = make_shared<ROSCameraHousingCalTInterface>(camera_optical_frame, 
								       camera_housing_frame,
								       camera_mounting_frame);
		}
		else if(transform_interface == std::string("ros_scti")){ 
		  (*camera_parameters)[i]["parent_frame"] >> camera_mounting_frame; 
		  temp_ti = make_shared<ROSSimpleCalTInterface>(camera_optical_frame,  camera_mounting_frame);
		}
		else if(transform_interface == std::string("ros_camera_scti")){ 
		  (*camera_parameters)[i]["parent_frame"] >> camera_mounting_frame; 
		  temp_ti = make_shared<ROSSimpleCameraCalTInterface>(camera_optical_frame,  camera_mounting_frame);
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
		scene_id = 0;
		ceres_blocks_.addMovingCamera(temp_camera, scene_id);

	      }
	  }
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
    std::ifstream target_input_file(target_def_file_name_.c_str());
    if (target_input_file.fail())
      {
	ROS_ERROR_STREAM(
			 "CalibrationJob::load(), couldn't open target_input_file: "
			 << target_def_file_name_.c_str());
	return (false);
      }
    Target temp_target;
    std::string temp_frame;
    std::string transform_interface;
    try
      {
	YAML::Parser target_parser(target_input_file);
	YAML::Node target_doc;
	target_parser.GetNextDocument(target_doc);
	// read in all static targets
	if (const YAML::Node *target_parameters = target_doc.FindValue("static_targets"))
	  {
	    ROS_DEBUG_STREAM("Found "<<target_parameters->size() <<" targets ");
	    for (unsigned int i = 0; i < target_parameters->size(); i++)
	      {
		// create shared target and transform interface
		shared_ptr<Target> temp_target = make_shared<Target>();
		shared_ptr<TransformInterface> temp_ti;

		temp_target->is_moving_ = false;
		(*target_parameters)[i]["target_name"] >> temp_target->target_name_;
		(*target_parameters)[i]["target_frame"] >> temp_target->target_frame_;
		(*target_parameters)[i]["target_type"] >> temp_target->target_type_;
		switch (temp_target->target_type_)
		  {
		  case pattern_options::Chessboard:
		    (*target_parameters)[i]["target_rows"] >> temp_target->checker_board_parameters_.pattern_rows;
		    (*target_parameters)[i]["target_cols"] >> temp_target->checker_board_parameters_.pattern_cols;
		    ROS_DEBUG_STREAM("TargetRows: "<<temp_target->checker_board_parameters_.pattern_rows);
		    break;
		  case pattern_options::CircleGrid:
		    (*target_parameters)[i]["target_rows"] >> temp_target->circle_grid_parameters_.pattern_rows;
		    (*target_parameters)[i]["target_cols"] >> temp_target->circle_grid_parameters_.pattern_cols;
		    (*target_parameters)[i]["circle_dia"]  >> temp_target->circle_grid_parameters_.circle_diameter;
		    temp_target->circle_grid_parameters_.is_symmetric=true;
		    ROS_DEBUG_STREAM("TargetRows: "<<temp_target->circle_grid_parameters_.pattern_rows);
		    break;
		  case pattern_options::ModifiedCircleGrid:
		    (*target_parameters)[i]["target_rows"] >> temp_target->circle_grid_parameters_.pattern_rows;
		    (*target_parameters)[i]["target_cols"] >> temp_target->circle_grid_parameters_.pattern_cols;
		    (*target_parameters)[i]["circle_dia"]  >> temp_target->circle_grid_parameters_.circle_diameter;
		    temp_target->circle_grid_parameters_.is_symmetric=true;
		    ROS_DEBUG_STREAM("TargetRows: "<<temp_target->circle_grid_parameters_.pattern_rows);
		    break;
		  case pattern_options::Balls:
		    // no parameters yet
		    break;
		  default:
		    ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard, CircleGrid, or ModifiedCircleGrid, Balls)");
		    return false;
		    break;
		  } // end of target type
		(*target_parameters)[i]["angle_axis_ax"] >> temp_target->pose_.ax;
		(*target_parameters)[i]["angle_axis_ay"] >> temp_target->pose_.ay;
		(*target_parameters)[i]["angle_axis_az"] >> temp_target->pose_.az;
		(*target_parameters)[i]["position_x"] >> temp_target->pose_.x;
		(*target_parameters)[i]["position_y"] >> temp_target->pose_.y;
		(*target_parameters)[i]["position_z"] >> temp_target->pose_.z;
		(*target_parameters)[i]["transform_interface"] >> transform_interface;

		// install target's transform interface
		if(transform_interface == std::string("ros_lti")){ 
		  temp_ti = make_shared<ROSListenerTransInterface>(temp_target->target_frame_);
		}
		else if(transform_interface == std::string("ros_bti")){ 
		  temp_ti = make_shared<ROSBroadcastTransInterface>(temp_target->target_frame_, temp_target->pose_);
		}
		else if(transform_interface == std::string("ros_scti")){ 
		  std::string parent_frame;
		  (*target_parameters)[i]["parent_frame"] >> parent_frame; 
		  temp_ti = make_shared<ROSSimpleCalTInterface>(temp_target->target_frame_,  parent_frame);
		}
		else if(transform_interface == std::string("ros_camera_scti")){ 
		  std::string parent_frame;
		  (*target_parameters)[i]["parent_frame"] >> parent_frame; 
		  temp_ti = make_shared<ROSSimpleCameraCalTInterface>(temp_target->target_frame_,  parent_frame);
		}
		else if(transform_interface == std::string("default_ti")){
		  temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
		}
		else{
		  ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
		  temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
		}
		temp_target->setTransformInterface(temp_ti);// install the transform interface 
		(*target_parameters)[i]["num_points"] >> temp_target->num_points_;
		const YAML::Node *points_node = (*target_parameters)[i].FindValue("points");
		if(temp_target->num_points_ != (int) points_node->size()){
		  ROS_ERROR("Expecting %d points found %d",temp_target->num_points_,(int) points_node->size());
		}
		ROS_DEBUG_STREAM("FoundPoints: "<<points_node->size());
		for (int j = 0; j < points_node->size(); j++)
		  {
		    const YAML::Node *pnt_node = (*points_node)[j].FindValue("pnt");
		    std::vector<float> temp_pnt;
		    (*pnt_node) >> temp_pnt;
		    Point3d temp_pnt3d;
		    temp_pnt3d.x = temp_pnt[0];
		    temp_pnt3d.y = temp_pnt[1];
		    temp_pnt3d.z = temp_pnt[2];
		    temp_target->pts_.push_back(temp_pnt3d);
		  }
		if(temp_target->is_moving_ == true){
		  ROS_ERROR("Static Target set to moving????");
		}
		ceres_blocks_.addStaticTarget(temp_target);
	      }
	  }

	// read in all moving targets
	if (const YAML::Node *target_parameters = target_doc.FindValue("moving_targets"))
	  {
	    ROS_DEBUG_STREAM("Found "<<target_parameters->size() <<"  moving targets ");
	    unsigned int scene_id=0;
	    for (unsigned int i = 0; i < target_parameters->size(); i++)
	      {
		// create shared target and transform interface
		shared_ptr<Target> temp_target = make_shared<Target>();
		shared_ptr<TransformInterface> temp_ti;

		temp_target->is_moving_ = true;
		(*target_parameters)[i]["target_name"] >> temp_target->target_name_;
		(*target_parameters)[i]["target_frame"] >> temp_target->target_frame_;
		(*target_parameters)[i]["target_type"] >> temp_target->target_type_;
		switch (temp_target->target_type_)
		  {
		  case pattern_options::Chessboard:
		    (*target_parameters)[i]["target_rows"] >> temp_target->checker_board_parameters_.pattern_rows;
		    (*target_parameters)[i]["target_cols"] >> temp_target->checker_board_parameters_.pattern_cols;
		    break;
		  case pattern_options::CircleGrid:
		    (*target_parameters)[i]["target_rows"] >> temp_target->circle_grid_parameters_.pattern_rows;
		    (*target_parameters)[i]["target_cols"] >> temp_target->circle_grid_parameters_.pattern_cols;
		    (*target_parameters)[i]["circle_dia"]  >> temp_target->circle_grid_parameters_.circle_diameter;
		    temp_target->circle_grid_parameters_.is_symmetric=true;
		    break;
		  case pattern_options::ModifiedCircleGrid:
		    (*target_parameters)[i]["target_rows"] >> temp_target->circle_grid_parameters_.pattern_rows;
		    (*target_parameters)[i]["target_cols"] >> temp_target->circle_grid_parameters_.pattern_cols;
		    (*target_parameters)[i]["circle_dia"]  >> temp_target->circle_grid_parameters_.circle_diameter;
		    temp_target->circle_grid_parameters_.is_symmetric=true;
		    break;
		  case pattern_options::Balls:
		    // no parameters yet
		    break;
		  default:
		    ROS_ERROR_STREAM("target_type does not correlate to a known pattern option (Chessboard or CircleGrid)");
		    return false;
		    break;
		  }

		// set initial pose of target
		(*target_parameters)[i]["angle_axis_ax"] >> temp_target->pose_.ax;
		(*target_parameters)[i]["angle_axis_ay"] >> temp_target->pose_.ay;
		(*target_parameters)[i]["angle_axis_az"] >> temp_target->pose_.az;
		(*target_parameters)[i]["position_x"] >> temp_target->pose_.x;
		(*target_parameters)[i]["position_y"] >> temp_target->pose_.y;
		(*target_parameters)[i]["position_z"] >> temp_target->pose_.z;
		(*target_parameters)[i]["transform_interface"] >> transform_interface;

		// install target's transform interface
		if(transform_interface == std::string("ros_lti")){ 
		  temp_ti = make_shared<ROSListenerTransInterface>(temp_target->target_frame_);
		}
		else if(transform_interface == std::string("ros_bti")){ 
		  temp_ti = make_shared<ROSBroadcastTransInterface>(temp_target->target_frame_, temp_target->pose_);
		}
		else if(transform_interface == std::string("ros_scti")){ 
		  std::string parent_frame;
		  (*target_parameters)[i]["parent_frame"] >> parent_frame; 
		  temp_ti = make_shared<ROSSimpleCalTInterface>(temp_target->target_frame_,  parent_frame);
		}
		else if(transform_interface == std::string("ros_camera_scti")){ 
		  std::string parent_frame;
		  (*target_parameters)[i]["parent_frame"] >> parent_frame; 
		  temp_ti = make_shared<ROSSimpleCameraCalTInterface>(temp_target->target_frame_,  parent_frame);
		}
		else if(transform_interface == std::string("default_ti")){
		  temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
		}
		else{
		  ROS_ERROR("Unimplemented Transform Interface: %s",transform_interface.c_str());
		  temp_ti = make_shared<DefaultTransformInterface>(temp_target->pose_);
		}
		temp_target->setTransformInterface(temp_ti);// install the transform interface 
		
		// set point for the target
		(*target_parameters)[i]["num_points"] >> temp_target->num_points_;
		const YAML::Node *points_node = (*target_parameters)[i].FindValue("points");
		if(temp_target->num_points_ != (int) points_node->size()){
		  ROS_ERROR("Expecting %d points found %d",temp_target->num_points_,(int) points_node->size());
		}
		for (int j = 0; j < points_node->size(); j++)
		  {
		    const YAML::Node *pnt_node = (*points_node)[j].FindValue("pnt");
		    std::vector<float> temp_pnt;
		    (*pnt_node) >> temp_pnt;
		    Point3d temp_pnt3d;
		    temp_pnt3d.x = temp_pnt[0];
		    temp_pnt3d.y = temp_pnt[1];
		    temp_pnt3d.z = temp_pnt[2];
		    temp_target->pts_.push_back(temp_pnt3d);
		  }
		ceres_blocks_.addMovingTarget(temp_target, scene_id);
	      }
	  }
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
    std::ifstream caljob_input_file(caljob_def_file_name_.c_str());
    if (caljob_input_file.fail())
      {
	ROS_ERROR_STREAM(
			 "ERROR CalibrationJob::load(), couldn't open caljob_input_file: "
			 << caljob_def_file_name_.c_str());
	return (false);
      }

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
	YAML::Parser caljob_parser(caljob_input_file);
	YAML::Node caljob_doc;
	caljob_parser.GetNextDocument(caljob_doc);

	caljob_doc["reference_frame"] >> reference_frame;
	ceres_blocks_.setReferenceFrame(reference_frame);
	caljob_doc["optimization_parameters"] >> opt_params;
	// read in all scenes
	if (const YAML::Node *caljob_scenes = caljob_doc.FindValue("scenes"))
	  {
	    ROS_DEBUG_STREAM("Found "<<caljob_scenes->size() <<" scenes");
	    scene_list_.resize(caljob_scenes->size() );
	    for (unsigned int i = 0; i < caljob_scenes->size(); i++)
	      {
		(*caljob_scenes)[i]["scene_id"] >> scene_id_num;

		(*caljob_scenes)[i]["trigger"] >> trigger_name;
		string ros_bool_param;
		string message;
		string server_name;
		boost::shared_ptr<Trigger> temp_trigger;

		// handle all the different trigger cases
		if(trigger_name == std::string("NO_WAIT_TRIGGER")){
		  temp_trigger = make_shared<NoWaitTrigger>();
		}
		else if(trigger_name == std::string("ROS_PARAM_TRIGGER")){
		  (*caljob_scenes)[i]["trig_param"] >> trig_param;
		  temp_trigger = make_shared<ROSParamTrigger>(trig_param);
		}
		else if(trigger_name == std::string("ROS_ACTION_TRIGGER")){
		  (*caljob_scenes)[i]["trig_action_server"] >> trig_action_server;
		  (*caljob_scenes)[i]["trig_action_msg"] >> trig_action_msg;
		  temp_trigger = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
		}
		else if(trigger_name == std::string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
		  (*caljob_scenes)[i]["trig_action_server"] >> trig_action_server;
		  std::vector<double>joint_values;
		  (*caljob_scenes)[i]["joint_values"] >> joint_values;
		  if(joint_values.size()<1){
		    ROS_ERROR("Couldn't read  joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
		  }
		  temp_trigger = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
		}
		else if(trigger_name == std::string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
		  (*caljob_scenes)[i]["trig_action_server"] >> trig_action_server;
		  Pose6d pose;
		  (*caljob_scenes)[i]["pose"][0] >> pose.x;
		  (*caljob_scenes)[i]["pose"][1] >> pose.y;
		  (*caljob_scenes)[i]["pose"][2] >> pose.z;
		  double qx,qy,qz,qw;
		  (*caljob_scenes)[i]["pose"][3] >> qx;
		  (*caljob_scenes)[i]["pose"][4] >> qy;
		  (*caljob_scenes)[i]["pose"][5] >> qz;
		  (*caljob_scenes)[i]["pose"][6] >> qw;
		  pose.setQuaternion(qx, qy, qz, qw);
		  temp_trigger = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
		}
		else if(trigger_name == std::string("ROS_CAMERA_OBSERVER_TRIGGER")){
		  const YAML::Node *trigger_parameters = (*caljob_scenes)[i].FindValue("trigger_parameters");
		  Roi roi;
		  std::string service_name;
		  std::string instructions;
		  std::string image_topic;
		  (*trigger_parameters)[0]["service_name"] >> service_name;
		  (*trigger_parameters)[0]["instructions"] >> instructions;
		  (*trigger_parameters)[0]["image_topic"] >> image_topic;
		  (*trigger_parameters)[0]["roi_min_x"] >> roi.x_min;
		  (*trigger_parameters)[0]["roi_max_x"] >> roi.x_max;
		  (*trigger_parameters)[0]["roi_min_y"] >> roi.y_min;
		  (*trigger_parameters)[0]["roi_max_y"] >> roi.y_max;
		  temp_trigger = make_shared<ROSCameraObserverTrigger>(service_name, instructions, image_topic, roi);
		}
		else{
		  ROS_ERROR("Unknown scene trigger type %s", trigger_name.c_str());
		}
		scene_list_.at(i).setTrigger(temp_trigger);

		scene_list_.at(i).setSceneId(scene_id_num);
		const YAML::Node *obs_node = (*caljob_scenes)[i].FindValue("observations");
		ROS_DEBUG_STREAM("Found "<<obs_node->size() <<" observations within scene "<<i);
		for (unsigned int j = 0; j < obs_node->size(); j++)
		  {
		    (*obs_node)[j]["camera"] >> camera_name;
		    (*obs_node)[j]["roi_x_min"] >> temp_roi.x_min;
		    (*obs_node)[j]["roi_x_max"] >> temp_roi.x_max;
		    (*obs_node)[j]["roi_y_min"] >> temp_roi.y_min;
		    (*obs_node)[j]["roi_y_max"] >> temp_roi.y_max;
		    (*obs_node)[j]["target"] >> target_name;
		    (*obs_node)[j]["cost_type"] >> cost_type_string;
		    cost_type = string2CostType(cost_type_string);
		    if((temp_cam = ceres_blocks_.getCameraByName(camera_name)) == NULL){
		      ROS_ERROR("Couldn't find camea %s",camera_name.c_str());
		    }
		    if((temp_targ = ceres_blocks_.getTargetByName(target_name)) == NULL){;
		      ROS_ERROR("Couldn't find target %s",target_name.c_str());
		    }
		    if(scene_id_num !=0 && temp_targ->is_moving_){ // if we have a moving target, we need to add it to the blocks
		      ceres_blocks_.addMovingTarget(temp_targ, scene_id_num);
		      temp_targ =  ceres_blocks_.getTargetByName(target_name, scene_id_num);
		    }
		    scene_list_.at(i).addCameraToScene(temp_cam);
		    cost_type = string2CostType(cost_type_string);
		    scene_list_.at(i).populateObsCmdList(temp_cam, temp_targ, temp_roi, cost_type);
		  }
	      }
	  }
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
    solved_ = runOptimization();
    if(solved_){
      pushTransforms(); // sends updated transforms to their intefaces
    }
    else{
      ROS_ERROR("Optimization failed");
    }
    return(solved_);
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

	current_scene.get_trigger()->waitForTrigger(); // this indicates scene is ready to capture

	pullTransforms(scene_id); // gets transforms of targets and cameras from their interfaces

	BOOST_FOREACH(shared_ptr<Camera> current_camera, current_scene.cameras_in_scene_)
	  {			// clear camera of existing observations
	    current_camera->camera_observer_->clearObservations(); // clear any recorded data
	    current_camera->camera_observer_->clearTargets(); // clear all targets
	  }

	BOOST_FOREACH(ObservationCmd o_command, current_scene.observation_command_list_)
	  {	// add each target and roi each camera's list of observations
	    
	    o_command.camera->camera_observer_->addTarget(o_command.target, o_command.roi, o_command.cost_type);
	  }

	BOOST_FOREACH( shared_ptr<Camera> current_camera, current_scene.cameras_in_scene_)
	  {// trigger the cameras
	    current_camera->camera_observer_->triggerCamera();
	  }

	// collect results
	P_BLOCK intrinsics;
	P_BLOCK extrinsics;
	P_BLOCK target_pose;
	P_BLOCK pnt_pos;
	std::string camera_name;
	std::string target_name;
	unsigned int  target_type;
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
		if(target_type == pattern_options::CircleGrid || target_type == pattern_options::ModifiedCircleGrid){
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
    // REMOVE ONCE DONE WITH NIST
    if(WRITE_DEBUG) writeObservationData(OUTPUT_DEBUG_FILE, observation_data_point_list_);

    // problem is declared here because we can't clear it as far as I can tell from the ceres documentation
    if(problem_ != NULL) {
      ROS_INFO("Deleting old problem.");
      delete(problem_); 
    }
    problem_ = new ceres::Problem; /*!< This is the object which solves non-linear optimization problems */

    total_observations_ =0;
    for(int i=0;i<observation_data_point_list_.size();i++){
      total_observations_ += observation_data_point_list_[i].items_.size();
    }
    if(total_observations_ == 0){ // TODO really need more than number of parameters being computed
      ROS_ERROR("Too few observations: %d",total_observations_);
      return(false);
    }
    
    ceres_blocks_.displayMovingCameras();

    // take all the data collected and create a Ceres optimization problem and run it
    ROS_INFO("Running Optimization with %d scenes",(int)scene_list_.size());
    ROS_DEBUG_STREAM("Optimizing "<<scene_list_.size()<<" scenes");
    BOOST_FOREACH(ObservationScene current_scene, scene_list_)
      {
	int scene_id = current_scene.get_id();
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
	    bool point_zero=false; // Set true to enable some debugging

	    if(0){// modify for debugging
	      if(point.x == 0.0 && point.y == 0.0 && point.z == 0.0){
		point_zero=true;
		//		ROS_ERROR("Observing Target Origin");
		//		showPose(target_pose_params, "target");
		target_pose.show("target_pose");
		showPose(extrinsics,"extrinsics");
		ROS_ERROR("u,v %6.3f %6.3f",image_x, image_y);
		//		showPose((P_BLOCK) &camera_mounting_pose.pb_pose[0], "camera_mounting_pose");
	      }
	    }

	    switch( ODP.cost_type_ ){
	    case cost_functions::CameraReprjErrorWithDistortion:
	      {
		CostFunction* cost_function =
		  CameraReprjErrorWithDistortion::Create(image_x, image_y);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
	      }
	      break;
	    case cost_functions::CameraReprjErrorWithDistortionPK:
	      {
		CostFunction* cost_function =
		  CameraReprjErrorWithDistortionPK::Create(image_x, image_y, 
							   point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics);
	      }
	      break;
	    case cost_functions::CameraReprjError:
	      {
		CostFunction* cost_function =
		  CameraReprjError::Create(image_x, image_y, 
					   focal_length_x, focal_length_y,
					   center_x, center_y);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, point.pb);
	      }
	      break;
	    case cost_functions::CameraReprjErrorPK:
	      {
		CostFunction* cost_function =
		  CameraReprjErrorPK::Create(image_x, image_y, 
					     focal_length_x, focal_length_y,
					     center_x, center_y,
					     point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics);
	      }
	      break;
	    case cost_functions::TargetCameraReprjError:
	      {
		CostFunction* cost_function =
		  TargetCameraReprjError::Create(image_x, image_y, 
						 focal_length_x, focal_length_y,
						 center_x, center_y);

		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
	      }
	      break;
	    case cost_functions::PosedTargetCameraReprjErrorPK:
	      {
		if(point_zero){
		  ROS_ERROR("u,v %f %f x,y,x %f %f %f", image_x, image_y, point.x, point.y, point.z);
		}
		CostFunction* cost_function =
		  PosedTargetCameraReprjErrorPK::Create(image_x, image_y, 
							focal_length_x,
							focal_length_y,
							center_x,
							center_y,
							target_pose,
							point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
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
		      
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
	      }
	      break;
	    case cost_functions::CircleCameraReprjErrorWithDistortion:
	      {
		CostFunction* cost_function =
		  CircleCameraReprjErrorWithDistortion::Create(image_x, image_y, circle_dia);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
	      }
	      break;
	    case cost_functions::CircleCameraReprjErrorWithDistortionPK:
	      {
		CostFunction* cost_function =
		  CircleCameraReprjErrorWithDistortionPK::Create(image_x, image_y,
								 circle_dia,
								 point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, point.pb);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics);
	      }
	      break;
	    case cost_functions::CircleTargetCameraReprjErrorWithDistortion:
	      {
		CostFunction* cost_function =
		  CircleTargetCameraReprjErrorWithDistortion::Create(image_x, image_y,
								     circle_dia);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, point.pb);
	      }
	      break;
	    case cost_functions::CircleTargetCameraReprjErrorWithDistortionPK:
	      {
		CostFunction* cost_function =
		  CircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, 
								       circle_dia,
								       point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, target_pose_params);
	      }
	      break;
	    case cost_functions::FixedCircleTargetCameraReprjErrorWithDistortionPK:
	      {
		CostFunction* cost_function =
		  FixedCircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, 
									    circle_dia,
									    point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics, target_pose_params);
	      }
	      break;
	    case cost_functions::SimpleCircleTargetCameraReprjErrorWithDistortionPK:
	      {
		CostFunction* cost_function =
		  SimpleCircleTargetCameraReprjErrorWithDistortionPK::Create(image_x, image_y, 
									     circle_dia,
									     point);
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, intrinsics);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params, point.pb);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics, target_pose_params);
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
		problem_->AddResidualBlock(cost_function, NULL , extrinsics);
		if(point_zero){
		  double residual[2];
		  double *params[1];
		  showPose(extrinsics,"extrinsics");
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
      }//for each scene
    ROS_INFO("total observations: %d ",total_observations_);
  
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  ceres::Solve(options, problem_, &ceres_summary_);

  if(ceres_summary_.termination_type == ceres::USER_SUCCESS
     || ceres_summary_.termination_type == ceres::FUNCTION_TOLERANCE
     || ceres_summary_.termination_type == ceres::GRADIENT_TOLERANCE
     || ceres_summary_.termination_type == ceres::PARAMETER_TOLERANCE
     ){
      ROS_INFO("Problem Solved");
      double error_per_observation = ceres_summary_.initial_cost/total_observations_;

      return true;
    }
    else{
      ROS_ERROR("Problem Not Solved termination type = %d success = %d", ceres_summary_.termination_type, ceres::USER_SUCCESS);
    }


    
  }//end runOptimization

 bool CalibrationJob::computeCovariance(std::vector<CovarianceVariableRequest> &variables, std::string &covariance_file_name)
  {
    FILE *fp;
    if((fp= fopen(covariance_file_name.c_str(), "w") ) != NULL){
      ceres::Covariance::Options covariance_options;
      covariance_options.algorithm_type = ceres::DENSE_SVD;
      ceres::Covariance covariance(covariance_options);
      std::vector<const double*> covariance_blocks;
      std::vector<int> block_sizes;
      std::vector<std::string> block_names;
      std::vector< std::pair< const double*, const double*> > covariance_pairs;

      BOOST_FOREACH(CovarianceVariableRequest req, variables){
	P_BLOCK intrinsics, extrinsics, pose_params;
	switch(req.request_type){
	case covariance_requests::StaticCameraIntrinsicParams:
	    intrinsics = ceres_blocks_.getStaticCameraParameterBlockIntrinsics(req.object_name.c_str());
	    covariance_blocks.push_back(intrinsics);
	    block_sizes.push_back(9);
	    block_names.push_back(req.object_name.c_str());
	    break;
	case covariance_requests::StaticCameraExtrinsicParams:
	  extrinsics = ceres_blocks_.getStaticCameraParameterBlockExtrinsics(req.object_name.c_str());
	  covariance_blocks.push_back(extrinsics);
	  block_sizes.push_back(6);
	  block_names.push_back(req.object_name.c_str());
	  break;
	case covariance_requests::MovingCameraIntrinsicParams:
	  intrinsics = ceres_blocks_.getMovingCameraParameterBlockIntrinsics(req.object_name.c_str());
	  covariance_blocks.push_back(intrinsics);
	  block_sizes.push_back(9);
	  block_names.push_back(req.object_name.c_str());
	  break;
	case covariance_requests::MovingCameraExtrinsicParams:
	  extrinsics = ceres_blocks_.getMovingCameraParameterBlockExtrinsics(req.object_name.c_str(), req.scene_id);
	  covariance_blocks.push_back(extrinsics);
	  block_sizes.push_back(6);
	  block_names.push_back(req.object_name.c_str());
	  break;
	case covariance_requests::StaticTargetPoseParams:
	  pose_params = ceres_blocks_.getStaticTargetPoseParameterBlock(req.object_name.c_str());
	  covariance_blocks.push_back(pose_params);
	  block_sizes.push_back(6);
	  block_names.push_back(req.object_name.c_str());
	  break;
	case covariance_requests::MovingTargetPoseParams:
	  pose_params = ceres_blocks_.getMovingTargetPoseParameterBlock(req.object_name.c_str(), req.scene_id);
	  covariance_blocks.push_back(pose_params);
	  block_sizes.push_back(6);
	  block_names.push_back(req.object_name.c_str());
	  break;
	default:
	  ROS_ERROR("unknown type of request");
	  return(false);
	  break;
	}// end of switch for request type
      }// end of for each request

      // create pairs from every combination of blocks in request
      for(int i=0; i<(int)covariance_blocks.size(); i++){
	for(int j=i; j<(int)covariance_blocks.size(); j++){
	  covariance_pairs.push_back(std::make_pair(covariance_blocks[i],covariance_blocks[j]) );
	}
      }
      covariance.Compute(covariance_pairs, problem_);

      fprintf(fp,"covariance blocks:\n");
      for(int i=0; i<(int)covariance_blocks.size(); i++){
	for(int j=i; j<(int)covariance_blocks.size(); j++){
	  fprintf(fp,"Cov[%s, %s]\n", block_names[i].c_str(), block_names[j].c_str());
	  int N = block_sizes[i];
	  int M = block_sizes[j];
	  double ij_cov_block[N*M];
	  covariance.GetCovarianceBlock(covariance_blocks[i], covariance_blocks[j], ij_cov_block);
	  for(int q=0; q<N;i++){
	    for(int k=0;k<M;k++){
	      double sigma_i = sqrt(ij_cov_block[q*N+q]);
	      double sigma_j = sqrt(ij_cov_block[k*N+k]);
	      if(q==k){
		fprintf(fp,"%6.3f ", sigma_i);
	      }
	      else{
		fprintf(fp,"%6.3lf ", ij_cov_block[q*N + k]/(sigma_i * sigma_j));
	      }
	    }// end of k loop
	    fprintf(fp,"\n");
	  }// end of q loop
	}// end of j loop
      }// end of i loop
      fclose(fp);
    }// end of if file opens
    else{
      ROS_ERROR("could not open covariance file %s", covariance_file_name.c_str());
      return(false);
    }
    return(true);
  } // end computeCovariance

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
  double CalibrationJob::finalCostPerObservation()
  {
    if(!solved_){
      ROS_ERROR("Can't call costPerObservation prior to solving");
      return(-1.0);
    }
    return(ceres_summary_.final_cost/total_observations_);
  }

  double CalibrationJob::initialCostPerObservation()
  {
    if(!solved_){
      ROS_ERROR("Can't call costPerObservation prior to solving");
      return(-1.0);
    }
    return(ceres_summary_.initial_cost/total_observations_);
  }

}//end namespace industrial_extrinsic_cal
