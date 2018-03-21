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
#include <industrial_extrinsic_cal/yaml_utils.h>

using std::ifstream;
using std::string;
using std::vector;
using boost::shared_ptr;
using YAML::Node;

namespace industrial_extrinsic_cal
{
bool parseCameras(std::string& cameras_input_file, vector<shared_ptr<Camera> >& cameras)
{
  Node camera_doc;
  bool rtn = true;
  try
  {
    Node camera_doc;
    if (!yamlNodeFromFileName(cameras_input_file, camera_doc))
    {
      ROS_ERROR("Can't parse yaml file %s", cameras_input_file.c_str());
    }
    cameras.clear();

    // read in all static cameras
    int n_static = 0;
    const YAML::Node& camera_parameters = parseNode(camera_doc, "static_cameras");
    if (camera_parameters)
    {
      for (unsigned int i = 0; i < camera_parameters.size(); i++)
      {
        shared_ptr<Camera> temp_camera = parseSingleCamera(camera_parameters[i]);
        cameras.push_back(temp_camera);
        n_static++;
      }
    }

    // read in all moving cameras
    int n_moving = 0;
    const YAML::Node& camera_parameters2 = parseNode(camera_doc, "moving_cameras");
    if (camera_parameters2)
    {
      for (unsigned int i = 0; i < camera_parameters2.size(); i++)
      {
        shared_ptr<Camera> temp_camera = parseSingleCamera(camera_parameters2[i]);
        temp_camera->is_moving_ = true;
        cameras.push_back(temp_camera);
        n_moving++;
      }
    }

    if (cameras.size() == 0)
    {
      ROS_ERROR_STREAM("No valid cameras found in file " << cameras_input_file.c_str());
      return false;
    }
    ROS_INFO_STREAM((int)cameras.size() << " cameras " << n_static << " static " << n_moving << " moving");
  }
  catch (YAML::ParserException& e)
  {
    ROS_ERROR_STREAM("Camera parsing failure " << e.what());
    rtn = false;
  }
  return (rtn);
}
shared_ptr<Camera> parseSingleCamera(const Node& node)
{
  shared_ptr<Camera> temp_camera;
  try
  {
    string temp_name, temp_topic, camera_optical_frame, camera_housing_frame, camera_mounting_frame, parent_frame;
    string trigger_name, transform_interface;
    CameraParameters temp_parameters;
    if (!parseString(node, "camera_name", temp_name)) ROS_ERROR("Can't read camera_name");
    if (!parseString(node, "trigger", trigger_name)) ROS_ERROR("Can't read trigger");
    if (!parseString(node, "image_topic", temp_topic)) ROS_ERROR("Can't read image_topic");
    if (!parseString(node, "camera_optical_frame", camera_optical_frame)) ROS_ERROR("Can't read camera_optical_frame");
    if (!parseString(node, "transform_interface", transform_interface)) ROS_ERROR("Can't read transform_interface");

    Pose6d pose;
    bool transform_available = parsePose(node, pose);
    if (transform_available)
    {
      temp_parameters.angle_axis[0] = pose.ax;
      temp_parameters.angle_axis[1] = pose.ay;
      temp_parameters.angle_axis[2] = pose.az;
      temp_parameters.position[0] = pose.x;
      temp_parameters.position[1] = pose.y;
      temp_parameters.position[2] = pose.z;
    }
    ROSCameraObserver camera_observer(temp_topic, temp_name);
    if (!camera_observer.pullCameraInfo(temp_parameters.focal_length_x, temp_parameters.focal_length_y,
                                        temp_parameters.center_x, temp_parameters.center_y,
                                        temp_parameters.distortion_k1, temp_parameters.distortion_k2,
                                        temp_parameters.distortion_k3, temp_parameters.distortion_p1,
                                        temp_parameters.distortion_p2, temp_parameters.width, temp_parameters.height))
    {
      ROS_WARN("Could not get camera info from topic, attempting to parse parameters from yaml file.");
      if (!parseDouble(node, "focal_length_x", temp_parameters.focal_length_x)) ROS_ERROR("Can't read focal_length_x");
      if (!parseDouble(node, "focal_length_y", temp_parameters.focal_length_y)) ROS_ERROR("Can't read focal_length_y");
      if (!parseDouble(node, "center_x", temp_parameters.center_x)) ROS_ERROR("Can't read center_x");
      if (!parseDouble(node, "center_y", temp_parameters.center_y)) ROS_ERROR("Can't read center_y");
      if (!parseDouble(node, "distortion_k1", temp_parameters.distortion_k1)) ROS_ERROR("Can't read distortion_k1");
      if (!parseDouble(node, "distortion_k2", temp_parameters.distortion_k2)) ROS_ERROR("Can't read distortion_k2");
      if (!parseDouble(node, "distortion_k3", temp_parameters.distortion_k3)) ROS_ERROR("Can't read distortion_k2");
      if (!parseDouble(node, "distortion_p1", temp_parameters.distortion_p1)) ROS_ERROR("Can't read distortion_p1");
      if (!parseDouble(node, "distortion_p2", temp_parameters.distortion_p2)) ROS_ERROR("Can't read distortion_p2");
      if (!parseInt(node, "image_height", temp_parameters.height)) ROS_ERROR("Can't read image_height");
      if (!parseInt(node, "image_width", temp_parameters.width)) ROS_ERROR("Can't read image_width");
    }

    // create a shared camera and a shared transform interface
    temp_camera = boost::make_shared<Camera>(temp_name, temp_parameters, false);
    temp_camera->trigger_ = parseTrigger(node, trigger_name);
    shared_ptr<TransformInterface> temp_ti =
        parseTransformInterface(node, transform_interface, camera_optical_frame, pose);
    temp_camera->setTransformInterface(temp_ti);  // install the transform interface
    if (transform_available)
    {
      ros::Duration(0.25).sleep();  // wait for listeners to wake up
      temp_camera->pushTransform();
    }
    else
    {
      temp_camera->pullTransform();
    }
    temp_camera->camera_observer_ = boost::make_shared<ROSCameraObserver>(temp_topic, temp_name);
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
    temp_trigger = boost::make_shared<NoWaitTrigger>();
  }
  else if (name == string("ROS_PARAM_TRIGGER"))
  {
    if (!parseString(node, "trig_param", trig_param)) ROS_ERROR("Can't read trig_param");
    temp_trigger = boost::make_shared<ROSParamTrigger>(trig_param);
  }
  else if (name == string("ROS_ACTION_TRIGGER"))
  {
    if (!parseString(node, "trig_action_server", trig_action_server)) ROS_ERROR("Can't read trig_action_server");
    if (!parseString(node, "trig_action_msg", trig_action_msg)) ROS_ERROR("Can't read trig_action_msg");
    temp_trigger = boost::make_shared<ROSActionServerTrigger>(trig_action_server, trig_action_msg);
  }
  else if (name == string("ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER"))
  {
    if (!parseString(node, "trig_action_server", trig_action_server)) ROS_ERROR("Can't read trig_action_server");
    std::vector<double> joint_values;
    if (!parseVectorD(node, "joint_values", joint_values)) ROS_ERROR("Can't read joint_values");
    if (joint_values.size() < 0)
    {
      ROS_ERROR("Couldn't read joint_values for ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER");
    }
    temp_trigger = boost::make_shared<ROSRobotJointValuesActionServerTrigger>(trig_action_server, joint_values);
  }
  else if (name == string("ROS_ROBOT_POSE_ACTION_TRIGGER"))
  {
    const YAML::Node& trig_node = parseNode(node, "trigger_parameters");
    if (!parseString(trig_node[0], "trig_action_server", trig_action_server))
      ROS_ERROR("Can't read trig_action_server");
    Pose6d pose;
    parsePose(trig_node, pose);
    temp_trigger = boost::make_shared<ROSRobotPoseActionServerTrigger>(trig_action_server, pose);
  }
  else if (name == string("ROS_CAMERA_OBSERVER_TRIGGER"))
  {
    const YAML::Node& trig_node = parseNode(node, "trigger_parameters");
    Roi roi;
    std::string service_name;
    std::string instructions;
    std::string image_topic;
    if (!parseString(trig_node[0], "service_name", service_name)) ROS_ERROR("Can't read service_name");
    if (!parseString(trig_node[0], "instructions", instructions)) ROS_ERROR("Can't read instructions");
    if (!parseString(trig_node[0], "image_topic", image_topic)) ROS_ERROR("Can't read image_topic");
    if (!parseInt(trig_node[0], "roi_min_x", roi.x_min)) ROS_ERROR("Can't read roi_min_x");
    if (!parseInt(trig_node[0], "roi_max_x", roi.x_max)) ROS_ERROR("Can't read roi_max_x");
    if (!parseInt(trig_node[0], "roi_min_y", roi.y_min)) ROS_ERROR("Can't read roi_min_y");
    if (!parseInt(trig_node[0], "roi_max_y", roi.y_max)) ROS_ERROR("Can't read roi_max_y");
    temp_trigger = boost::make_shared<ROSCameraObserverTrigger>(service_name, instructions, image_topic, roi);
  }
  else
  {
    ROS_ERROR("No scene trigger of type %s", name.c_str());
  }
  return (temp_trigger);
}

shared_ptr<TransformInterface> parseTransformInterface(const Node& node, std::string& name, std::string& frame,
                                                       Pose6d& pose)
{
  shared_ptr<TransformInterface> temp_ti;
  std::string camera_housing_frame;
  std::string camera_mounting_frame;
  if (name == std::string("ros_lti"))
  {  // this option makes no sense for a camera
    temp_ti = boost::make_shared<ROSListenerTransInterface>(frame);
  }
  else if (name == std::string("ros_bti"))
  {  // this option makes no sense for a camera
    temp_ti = boost::make_shared<ROSBroadcastTransInterface>(frame);
  }
  else if (name == std::string("ros_camera_lti"))
  {
    temp_ti = boost::make_shared<ROSCameraListenerTransInterface>(frame);
  }
  else if (name == std::string("ros_camera_bti"))
  {
    temp_ti = boost::make_shared<ROSCameraBroadcastTransInterface>(frame);
  }
  else if (name == std::string("ros_camera_housing_lti"))
  {
    if (!parseString(node, "camera_housing_frame", camera_housing_frame)) ROS_ERROR("Can't read camera_housing_frame");
    temp_ti = boost::make_shared<ROSCameraHousingListenerTInterface>(frame, camera_housing_frame);
  }
  else if (name == std::string("ros_camera_housing_bti"))
  {
    if (!parseString(node, "camera_housing_frame", camera_housing_frame)) ROS_ERROR("Can't read camera_housing_frame");
    if (!parseString(node, "camera_mounting_frame", camera_mounting_frame))
      ROS_ERROR("Can't read camera_mounting_frame");
    temp_ti = boost::make_shared<ROSCameraHousingBroadcastTInterface>(frame, camera_housing_frame,
                                                                      camera_mounting_frame, pose);
  }
  else if (name == std::string("ros_camera_housing_cti"))
  {
    if (!parseString(node, "camera_housing_frame", camera_housing_frame)) ROS_ERROR("Can't read camera_housing_frame");
    if (!parseString(node, "camera_mounting_frame", camera_mounting_frame))
      ROS_ERROR("Can't read camera_mounting_frame");
    temp_ti = boost::make_shared<ROSCameraHousingCalTInterface>(frame, camera_housing_frame, camera_mounting_frame);
  }
  else if (name == std::string("ros_scti"))
  {
    if (!parseString(node, "parent_frame", camera_mounting_frame)) ROS_ERROR("Can't read parent_frame");
    temp_ti = boost::make_shared<ROSSimpleCalTInterface>(frame, camera_mounting_frame);
  }
  else if (name == std::string("ros_camera_scti"))
  {
    if (!parseString(node, "parent_frame", camera_mounting_frame)) ROS_ERROR("Can't read parent_frame");
    temp_ti = boost::make_shared<ROSSimpleCameraCalTInterface>(frame, camera_mounting_frame);
  }
  else if (name == std::string("default_ti"))
  {
    temp_ti = boost::make_shared<DefaultTransformInterface>();
  }
  else
  {
    ROS_ERROR("Unimplemented Transform Interface: %s", name.c_str());
    temp_ti = boost::make_shared<DefaultTransformInterface>();
  }
  return (temp_ti);
}

bool parsePose(const Node& node, Pose6d& pose)
{
  bool rtn = true;
  std::vector<double> temp_values;
  if (parseVectorD(node, "xyz_quat_pose", temp_values))
  {
    if (temp_values.size() == 7)
    {
      pose.x = temp_values[0];
      pose.y = temp_values[1];
      pose.z = temp_values[2];
      pose.setQuaternion(temp_values[3], temp_values[4], temp_values[5], temp_values[6]);
    }  // got 7 values
    else
    {  // not 7 values
      ROS_ERROR("did not read xyz_quat_pose correctly, %d values, 7 required", (int)temp_values.size());
      rtn = false;
    }  // right number of values
  }    // "xyz_quat_pose" exists in yaml node
  else if (parseVectorD(node, "xyz_aaxis_pose", temp_values))
  {
    if (temp_values.size() == 6)
    {
      pose.x = temp_values[0];
      pose.y = temp_values[1];
      pose.z = temp_values[2];
      pose.setAngleAxis(temp_values[3], temp_values[4], temp_values[5]);
    }  // got 6 values
    else
    {  // can't find xyz_aaxis_pose
      ROS_ERROR("did not read xyz_aaxis_pose correctly, %d values, 6 required v[0] = %lf", (int)temp_values.size(),
                temp_values[0]);
      rtn = false;
    }
  }
  else
  {  // cant read xyz_aaxis either
    rtn = false;
  }
  return rtn;
}

}  // end of industrial_extrinsic_cal namespace
