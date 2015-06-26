#include <stdlib.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/ros_triggers.h>
#include <industrial_extrinsic_cal/camera_yaml_parser.h>

using std::ifstream;
using std::string;
using std::vector;
using boost::shared_ptr;
using boost::make_shared;
using YAML::Node;

namespace industrial_extrinsic_cal {

  bool parseCameras(ifstream &cameras_input_file,vector<shared_ptr<Camera> >& cameras)
  {
    YAML::Parser camera_parser(cameras_input_file);
    Node camera_doc;
    bool rtn=true;
    try{
      camera_parser.GetNextDocument(camera_doc);
      
      // read in all static cameras
      cameras.clear();
      if (const Node *camera_parameters = parseNode(camera_doc, "static_cameras") ){
	ROS_INFO_STREAM("Found "<<camera_parameters->size()<<" static cameras ");
	for (unsigned int i = 0; i < camera_parameters->size(); i++){
	  shared_ptr<Camera> temp_camera = parseSingleCamera((*camera_parameters)[i]);
	  cameras.push_back(temp_camera);
	}
      } // end if there are any cameras in file
      // read in all moving cameras
      if (const Node *camera_parameters = parseNode(camera_doc, "moving_cameras")){
	ROS_INFO_STREAM("Found "<<camera_parameters->size()<<" moving cameras ");
	for (unsigned int i = 0; i < camera_parameters->size(); i++){
	  shared_ptr<Camera> temp_camera = parseSingleCamera((*camera_parameters)[i]);
	  temp_camera->is_moving_ = true;
	  cameras.push_back(temp_camera);
	}
      } // end if there are any movingcameras in file
      ROS_INFO_STREAM("Successfully read in " << (int) cameras.size() << " cameras");
    }
    catch (YAML::ParserException& e){
      ROS_ERROR("Camera parsing failure");
      rtn = false;
    }
    return(rtn);
  }
  shared_ptr<Camera> parseSingleCamera(const Node &node)
  {
    shared_ptr<Camera> temp_camera;
    try{
      string temp_name, temp_topic, camera_optical_frame, camera_housing_frame, camera_mounting_frame, parent_frame;
      string trigger_name, transform_interface;
      CameraParameters temp_parameters;
      parseString(node, "camera_name", temp_name);
      parseString(node, "trigger", trigger_name);
      parseString(node, "image_topic", temp_topic);
      parseString(node, "camera_optical_frame", camera_optical_frame);
      parseString(node, "transform_interface", transform_interface);

      bool transform_available=true;       /* need to know if camera parameters are defined in YAML */
      transform_available &= parseDouble(node, "angle_axis_ax", temp_parameters.angle_axis[0]);
      transform_available &= parseDouble(node, "angle_axis_ay", temp_parameters.angle_axis[1]);
      transform_available &= parseDouble(node, "angle_axis_az", temp_parameters.angle_axis[2]);
      transform_available &= parseDouble(node, "position_x", temp_parameters.position[0]);
      transform_available &= parseDouble(node, "position_y", temp_parameters.position[1]);
      transform_available &= parseDouble(node, "position_z", temp_parameters.position[2]);

      parseDouble(node, "focal_length_x", temp_parameters.focal_length_x);
      parseDouble(node, "focal_length_y", temp_parameters.focal_length_y);
      parseDouble(node, "center_x", temp_parameters.center_x);
      parseDouble(node, "center_y", temp_parameters.center_y);
      parseDouble(node, "distortion_k1", temp_parameters.distortion_k1);
      parseDouble(node, "distortion_k2", temp_parameters.distortion_k2);
      parseDouble(node, "distortion_k3", temp_parameters.distortion_k3);
      parseDouble(node, "distortion_p1", temp_parameters.distortion_p1);
      parseDouble(node, "distortion_p2", temp_parameters.distortion_p2);
      parseInt(node, "image_height", temp_parameters.height);
      parseInt(node, "image_width", temp_parameters.width);
    
      // create a shared camera and a shared transform interface
      temp_camera = make_shared<Camera>(temp_name, temp_parameters, false);
      temp_camera->trigger_ = parseTrigger(node, trigger_name);
      shared_ptr<TransformInterface>  temp_ti = parseTransformInterface(node, transform_interface, camera_optical_frame);
      temp_camera->setTransformInterface(temp_ti);// install the transform interface 
      if(transform_available){
	temp_camera->pushTransform();
      }
      else{
	temp_camera->pullTransform();
      }
      temp_camera->camera_observer_ = make_shared<ROSCameraObserver>(temp_topic);
    }
    catch (YAML::ParserException& e){
      ROS_INFO_STREAM("Failed to read in moving cameras from yaml file ");
      ROS_INFO_STREAM("camera name    = " << temp_camera->camera_name_.c_str());
      ROS_INFO_STREAM("angle_axis_ax  = " << temp_camera->camera_parameters_.angle_axis[0]);
      ROS_INFO_STREAM("angle_axis_ay  = " << temp_camera->camera_parameters_.angle_axis[1]);
      ROS_INFO_STREAM("angle_axis_az  = " << temp_camera->camera_parameters_.angle_axis[2]);
      ROS_INFO_STREAM("position_x     = " << temp_camera->camera_parameters_.position[0]);
      ROS_INFO_STREAM("position_y     = " << temp_camera->camera_parameters_.position[1]);
      ROS_INFO_STREAM("position_z     = " << temp_camera->camera_parameters_.position[2]);
      ROS_INFO_STREAM("focal_length_x = " << temp_camera->camera_parameters_.focal_length_x);
      ROS_INFO_STREAM("focal_length_y = " << temp_camera->camera_parameters_.focal_length_y);
      ROS_INFO_STREAM("center_x       = " << temp_camera->camera_parameters_.center_x);
      ROS_INFO_STREAM("center_y       = " << temp_camera->camera_parameters_.center_y);
    }
    return(temp_camera);
  }
  
  shared_ptr<Trigger> parseTrigger(const Node &node, string &name)
  {
    shared_ptr<Trigger> temp_trigger;
    std::string trig_param;
    std::string trig_action_server;
    std::string trig_action_msg;
    // handle all the different trigger cases
    if(name == string("NO_WAIT_TRIGGER")){
      temp_trigger = make_shared<NoWaitTrigger>();
    }
    else if(name == string("ROS_PARAM_TRIGGER")){
      parseString(node, "trig_param", trig_param);
      temp_trigger = make_shared<ROSParamTrigger>(trig_param);
    }
    else if(name == string("ROS_ACTION_TRIGGER")){
      parseString(node, "trig_action_server", trig_action_server);
      parseString(node, "trig_action_msg", trig_action_msg);
      temp_trigger = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
    }
    else if(name == string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER")){
      parseString(node, "trig_action_server", trig_action_server);
      std::vector<double>joint_values;
      parseVectorD(node, "joint_values", joint_values);
      if(joint_values.size()<0){
	ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
      }
      temp_trigger = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
    }
    else if(name == string("ROS_ROBOT_POSE_ACTION_TRIGGER")){
      parseString(node, "trig_action_server", trig_action_server);
      Pose6d pose;
      parsePose(node, pose);
      temp_trigger = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
    }
    else{
      ROS_ERROR("No scene trigger of type %s", name.c_str());
    }
    return(temp_trigger);
  }
  
  shared_ptr<TransformInterface> parseTransformInterface(const Node &node, std::string &name, std::string &frame)
  {
    shared_ptr<TransformInterface> temp_ti;
    std::string camera_housing_frame;
    std::string camera_mounting_frame;
    if(name == std::string("ros_lti")){ // this option makes no sense for a camera
      temp_ti = make_shared<ROSListenerTransInterface>(frame);
    }
    else if(name == std::string("ros_bti")){ // this option makes no sense for a camera
      temp_ti = make_shared<ROSBroadcastTransInterface>(frame);
    }
    else if(name == std::string("ros_camera_lti")){ 
      temp_ti = make_shared<ROSCameraListenerTransInterface>(frame);
    }
    else if(name == std::string("ros_camera_bti")){ 
      temp_ti = make_shared<ROSCameraBroadcastTransInterface>(frame);
    }
    else if(name == std::string("ros_camera_housing_lti")){ 
      parseString(node, "camera_housing_frame", camera_housing_frame);
      temp_ti = make_shared<ROSCameraHousingListenerTInterface>(frame,camera_housing_frame);
    }
    else if(name == std::string("ros_camera_housing_bti")){ 
      parseString(node, "camera_housing_frame", camera_housing_frame); 
      parseString(node, "camera_mounting_frame", camera_mounting_frame);
      temp_ti = make_shared<ROSCameraHousingBroadcastTInterface>(frame, 
								 camera_housing_frame,
								 camera_mounting_frame);
    }
    else if(name == std::string("ros_camera_housing_cti")){ 
      parseString(node, "camera_housing_frame", camera_housing_frame); 
      parseString(node, "camera_mounting_frame", camera_mounting_frame);
      temp_ti = make_shared<ROSCameraHousingCalTInterface>(frame, 
							   camera_housing_frame,
							   camera_mounting_frame);
    }
    else if(name == std::string("ros_scti")){ 
      parseString(node, "parent_frame", camera_mounting_frame);
      temp_ti = make_shared<ROSSimpleCalTInterface>(frame,  camera_mounting_frame);
    }
    else if(name == std::string("ros_camera_scti")){ 
      parseString(node, "parent_frame", camera_mounting_frame);
      temp_ti = make_shared<ROSSimpleCameraCalTInterface>(frame,  camera_mounting_frame);
    }
    else if(name == std::string("default_ti")){
      temp_ti = make_shared<DefaultTransformInterface>();
    }
    else{
      ROS_ERROR("Unimplemented Transform Interface: %s",name.c_str());
      temp_ti = make_shared<DefaultTransformInterface>();
    }
    return(temp_ti);
  }

  bool parsePose(const Node &node, Pose6d &pose)
  {
    bool rtn = true;
    std::vector<double> temp_values;
    if(parseVectorD(node, "pose", temp_values)){
      if(temp_values.size() != 7){
	ROS_ERROR("did not read pose correctly, %d values, 7 required", (int) temp_values.size());
	rtn = false;
      }
      else{
	pose.x = temp_values[0];
	pose.y = temp_values[1];
	pose.z = temp_values[2];
	double qx,qy,qz,qw;
	qx = temp_values[3];
	qy = temp_values[4];
	qz = temp_values[5];
	qw = temp_values[6];
	pose.setQuaternion(qx, qy, qz, qw);
      }// right number of values
    }// "pose" exists in yaml node
    else{
      rtn = false;
    }
    return rtn ;
  }

}// end of industrial_extrinsic_cal namespace
