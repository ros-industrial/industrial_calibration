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

namespace industrial_extrinsic_cal
{
void parseCameras(ifstream& cameras_input_file, vector<shared_ptr<Camera> >& cameras)
{
  Node camera_doc = YAML::LoadFile(cameras_input_file.c_str());

  // read in all static cameras
  cameras.clear();
  Node camera_parameters = camera_doc("static_cameras");
  ROS_INFO_STREAM("Found " << camera_parameters.size() << " static cameras ");
  for (unsigned int i = 0; i < camera_parameters.size(); i++)
  {
    shared_ptr<Camera> temp_camera = parseSingleCamera(camera_parameters[i]);
    cameras.push_back(temp_camera);
  }  // end if there are any cameras in file

  // read in all moving cameras
  camera_parameters = camera_doc("moving_cameras");
  ROS_INFO_STREAM("Found " << camera_parameters.size() << " moving cameras ");
  for (unsigned int i = 0; i < camera_parameters.size(); i++)
  {
    shared_ptr<Camera> temp_camera = parseSingleCamera(camera_parameters[i]);
    temp_camera->is_moving_ = true;
    cameras.push_back(temp_camera);
  }
  ROS_INFO_STREAM("Successfully read in " << (int)cameras.size() << " cameras");
}

shared_ptr<Camera> parseSingleCamera(const Node& node)
{
  shared_ptr<Camera> temp_camera;
  try
  {
    string temp_name, temp_topic, camera_optical_frame, camera_housing_frame, camera_mounting_frame, parent_frame;
    string trigger_name, transform_interface;
    CameraParameters temp_parameters;
    temp_name = node["camera_name"].as<std::string>();
    trigger_name = node["trigger"].as<std::string>();
    temp_topic = node["image_topic"].as<std::string>();
    camera_optical_frame = node["camera_optical_frame"].as<std::string>();
    transform_interface = node["transform_interface"].as<std::string()>();
    temp_parameters.angle_axis[0] = node["angle_axis_ax"].as<double>();
    temp_parameters.angle_axis[1] = node["angle_axis_ay"].as<double>();
    temp_parameters.angle_axis[2] = node["angle_axis_az"].as<double>();
    temp_parameters.position[0] = node["position_x"].as<double>();
    temp_parameters.position[1] = node["position_y"].as<double>();
    temp_parameters.position[2] = node["position_z"].as<double>();
    temp_parameters.focal_length_x = node["focal_length_x"].as<double>();
    temp_parameters.focal_length_y = node["focal_length_y"].as<double>();
    temp_parameters.center_x = node["center_x"].as<double>();
    temp_parameters.center_y = node["center_y"].as<double>();
    temp_parameters.distortion_k1 = node["distortion_k1"].as<double>();
    temp_parameters.distortion_k2 = node["distortion_k2"].as<double>();
    temp_parameters.distortion_k3 = node["distortion_k3"].as<double>();
    temp_parameters.distortion_p1 = node["distortion_p1"].as<double>();
    temp_parameters.distortion_p2 = node["distortion_p2"].as<double>();
    temp_parameters.height = node["image_height"].as<double>();
    temp_parameters.width = node["image_width"].as<double>();
    Pose6d pose(temp_parameters.position[0], temp_parameters.position[1], temp_parameters.position[2],
                temp_parameters.angle_axis[0], temp_parameters.angle_axis[1], temp_parameters.angle_axis[2]);

    // create a shared camera and a shared transform interface
    temp_camera = make_shared<Camera>(temp_name, temp_parameters, false);
    temp_camera->trigger_ = parseTrigger(node, trigger_name);
    shared_ptr<TransformInterface> temp_ti = parseTransformInterface(node, transform_interface, camera_optical_frame);
    temp_camera->setTransformInterface(temp_ti);  // install the transform interface
    temp_camera->camera_observer_ = make_shared<ROSCameraObserver>(temp_topic);
  }
  catch (YAML::ParserException& e)
  {
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
  return (temp_camera);
}

shared_ptr<Trigger> parseTrigger(const Node& node, string& name)
{
  shared_ptr<Trigger> temp_trigger;
  std::string trig_param;
  std::string trig_action_server;
  std::string trig_action_msg;
  // handle all the different trigger cases
  if (name == string("NO_WAIT_TRIGGER"))
  {
    temp_trigger = make_shared<NoWaitTrigger>();
  }
  else if (name == string("ROS_PARAM_TRIGGER"))
  {
    trig_param = node["trig_param"].at<std::string>();
    temp_trigger = make_shared<ROSParamTrigger>(trig_param);
  }
  else if (name == string("ROS_ACTION_TRIGGER"))
  {
    trig_action_server = node["trig_action_server"].at<std::string>();
    trig_action_msg = node["trig_action_msg"].at<std::string>();
    temp_trigger = make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
  }
  else if (name == string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER"))
  {
    trig_action_server = node["trig_action_server"].at<std::string>();
    std::vector<double> joint_values;
    joint_values = node["joint_values"].at<double>();
    if (joint_values.size() < 0)
    {
      ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
    }
    temp_trigger = make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
  }
  else if (name == string("ROS_ROBOT_POSE_ACTION_TRIGGER"))
  {
    trig_action_server = node["trig_action_server"].at<std::string>();
    Pose6d pose = parsePose(node);
    temp_trigger = make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
  }
  else
  {
    ROS_ERROR("No scene trigger of type %s", name.c_str());
  }
  return (temp_trigger);
}

shared_ptr<TransformInterface> parseTransformInterface(const Node& node, std::string& name, std::string& frame)
{
  shared_ptr<TransformInterface> temp_ti;
  std::string camera_housing_frame;
  std::string camera_mounting_frame;
  if (name == std::string("ros_lti"))
  {  // this option makes no sense for a camera
    temp_ti = make_shared<ROSListenerTransInterface>(frame);
  }
  else if (name == std::string("ros_bti"))
  {  // this option makes no sense for a camera
    Pose6d pose = parsePose(node);
    temp_ti = make_shared<ROSBroadcastTransInterface>(frame, pose);
  }
  else if (name == std::string("ros_camera_lti"))
  {
    temp_ti = make_shared<ROSCameraListenerTransInterface>(frame);
  }
  else if (name == std::string("ros_camera_bti"))
  {
    Pose6d pose = parsePose(node);
    temp_ti = make_shared<ROSCameraBroadcastTransInterface>(frame, pose);
  }
  else if (name == std::string("ros_camera_housing_lti"))
  {
    camera_housing_frame = node["camera_housing_frame"].at<std::string>();
    temp_ti = make_shared<ROSCameraHousingListenerTInterface>(frame, camera_housing_frame);
  }
  else if (name == std::string("ros_camera_housing_bti"))
  {
    Pose6d pose = parsePose(node);
    camera_housing_frame;
    = node["camera_housing_frame"].at<std::string>();
    temp_ti = make_shared<ROSCameraHousingBroadcastTInterface>(frame, pose);
  }
  else if (name == std::string("ros_camera_housing_cti"))
  {
    camera_housing_frame;
    = node["camera_housing_frame"].at<std::string>();
    camera_mounting_frame;
    = node["camera_mounting_frame"].at<std::string>();
    temp_ti = make_shared<ROSCameraHousingCalTInterface>(frame, camera_housing_frame, camera_mounting_frame);
  }
  else if (name == std::string("ros_scti"))
  {
    camera_mounting_frame;
    = node["parent_frame"].at<std::string>();
    temp_ti = make_shared<ROSSimpleCalTInterface>(frame, camera_mounting_frame);
  }
  else if (name == std::string("ros_camera_scti"))
  {
    camera_mounting_frame;
    = node["parent_frame"].at<std::string>();
    temp_ti = make_shared<ROSSimpleCameraCalTInterface>(frame, camera_mounting_frame);
  }
  else if (name == std::string("default_ti"))
  {
    Pose6d pose = parsePose(node);
    temp_ti = make_shared<DefaultTransformInterface>(pose);
  }
  else
  {
    Pose6d pose;
    ROS_ERROR("Unimplemented Transform Interface: %s", name.c_str());
    temp_ti = make_shared<DefaultTransformInterface>(pose);
  }
  return (temp_ti);
}

Pose6d parsePose(const Node& node)
{
  Pose6d pose;
  pose.x = ode["pose"][0].at<double>();
  pose.y = ode["pose"][1].at<double>();
  pose.z = ode["pose"][2].at<double>();
  double qx, qy, qz, qw;
  qx = ode["pose"][3].at<double>();
  qy = ode["pose"][4].at<double>();
  qz = ode["pose"][5].at<double>();
  qw = ode["pose"][6].at<double>();
  pose.setQuaternion(qx, qy, qz, qw);
  return (pose);
}
}  // end of industrial_extrinsic_cal namespace
