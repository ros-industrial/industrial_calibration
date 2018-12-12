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

#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <iostream>
#include <fstream>
namespace industrial_extrinsic_cal
{

using std::string;

ROSListenerTransInterface::ROSListenerTransInterface(const string& transform_frame)
{
  transform_frame_ = transform_frame;
  transform_.child_frame_id_ = transform_frame_;
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
}

Pose6d ROSListenerTransInterface::pullTransform()
{
  if (!ref_frame_initialized_)
  {
    Pose6d pose(0, 0, 0, 0, 0, 0);
    ROS_ERROR("Trying to pull transform from interface without setting ref frame ROSListenerTransInterface");
    return (pose);
  }
  else
  {
    pose_ = getPoseFromTF(ref_frame_, transform_frame_, tf_listener_);
    return (pose_);
  }
}

ROSCameraListenerTransInterface::ROSCameraListenerTransInterface(const string& transform_frame, const string& camera_mounting_frame):
  ROSCameraTransformInterface(transform_frame, camera_mounting_frame, "")
{
  transform_.child_frame_id_ = transform_frame_;
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
}

Pose6d ROSCameraListenerTransInterface::pullTransform()
{
    pose_ = getPoseFromTF(transform_frame_, mounting_frame_, tf_listener_);
    return (pose_);
}

/** @brief this object is intened to be used for cameras not targets
 *            It simply listens to a pose from camera's optical frame to reference frame, this must be set in a urdf
 *            This is the inverse of the transform from world to camera's optical frame
 *            push does nothing
 *            store does nothing
 */
ROSCameraHousingListenerTInterface::ROSCameraHousingListenerTInterface(const string& transform_frame,
								       const string& mounting_frame,
                                                                       const string& housing_frame):
  ROSCameraTransformInterface(transform_frame, mounting_frame, housing_frame)
{
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
}

Pose6d ROSCameraHousingListenerTInterface::pullTransform()
{
    pose_ = getPoseFromTF(transform_frame_, mounting_frame_, tf_listener_);
    return (pose_);
}

ROSBroadcastTransInterface::ROSBroadcastTransInterface(const string& transform_frame)
{
  transform_frame_ = transform_frame;
  transform_.child_frame_id_ = transform_frame_;
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
}

bool ROSBroadcastTransInterface::pushTransform(Pose6d& pose)
{
  pose_ = pose;
  if (!ref_frame_initialized_)
  {
    return (false);  // timer won't start publishing until ref_frame_ is defined
  }
  return (true);
}

bool ROSBroadcastTransInterface::store(string& filePath)
{
  std::ofstream outputFile(filePath.c_str(), std::ios::app);  // open for appending
  if (outputFile.is_open())
  {
    double qx, qy, qz, qw;
    pose_.getQuaternion(qx, qy, qz, qw);
    outputFile << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"";
    outputFile << transform_frame_ << "_tf_broadcaster"
               << "\" args=\"";
    outputFile << pose_.x << ' ' << pose_.y << ' ' << pose_.z << ' ';
    outputFile << qx << ' ' << qy << ' ' << qz << ' ' << qw;
    outputFile << " " << ref_frame_;
    outputFile << " " << transform_frame_;
    outputFile << " 100\" />" << std::endl;
    outputFile.close();
    return (true);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file:" << filePath);
    return false;
  }  // end if writing to file
}

void ROSBroadcastTransInterface::setReferenceFrame(string& ref_frame)
{
  static ros::NodeHandle nh;
  ref_frame_ = ref_frame;
  ref_frame_initialized_ = true;
  timer_ = nh.createTimer(ros::Rate(1.0), &ROSBroadcastTransInterface::timerCallback, this);
}

void ROSBroadcastTransInterface::timerCallback(const ros::TimerEvent& timer_event)
{  // broadcast current value of pose as a transform each time called
  transform_.setBasis(pose_.getBasis());
  transform_.setOrigin(pose_.getOrigin());
  transform_.child_frame_id_ = transform_frame_;
  transform_.frame_id_ = ref_frame_;
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), ref_frame_, transform_frame_));
}

ROSCameraBroadcastTransInterface::ROSCameraBroadcastTransInterface(const string& transform_frame, const string& mounting_frame) :
  ROSCameraTransformInterface(transform_frame, mounting_frame, "")
{
  transform_.child_frame_id_ = transform_frame_;
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
}

bool ROSCameraBroadcastTransInterface::pushTransform(Pose6d& pose)
{
  pose_ = pose;
  if (!ref_frame_initialized_)
  {
    return (false);  // timer won't start publishing until ref_frame_ is defined
  }
  return (true);
}

bool ROSCameraBroadcastTransInterface::store(string& filePath)
{
  std::ofstream outputFile(filePath.c_str(), std::ios::app);  // open for appending
  if (outputFile.is_open())
  {
    double qx, qy, qz, qw;
    pose_.getInverse().getQuaternion(qx, qy, qz, qw);
    outputFile << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"";
    outputFile << transform_frame_ << "_tf_broadcaster"
               << "\" args=\"";
    outputFile << pose_.getInverse().x << ' ' << pose_.getInverse().y << ' ' << pose_.getInverse().z << ' ';
    outputFile << qx << ' ' << qy << ' ' << qz << ' ' << qw;
    outputFile << " " << mounting_frame_;
    outputFile << " " << transform_frame_;
    outputFile << " 100\" />" << std::endl;
    outputFile.close();
    return (true);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file:" << filePath);
    return false;
  }  // end if writing to file
}

void ROSCameraBroadcastTransInterface::setReferenceFrame(string& ref_frame)
{
  static ros::NodeHandle nh;
  ref_frame_ = ref_frame;
  ref_frame_initialized_ = true;
  timer_ = nh.createTimer(ros::Rate(1.0), &ROSCameraBroadcastTransInterface::timerCallback, this);
}

void ROSCameraBroadcastTransInterface::timerCallback(const ros::TimerEvent& timer_event)
{  // broadcast current value of pose.inverse() as a transform each time called
  transform_.setBasis(pose_.getInverse().getBasis());
  transform_.setOrigin(pose_.getInverse().getOrigin());
  transform_.child_frame_id_ = transform_frame_;
  transform_.frame_id_ = mounting_frame_;
  //    ROS_INFO("broadcasting %s in %s",transform_frame_.c_str(),ref_frame_.c_str());
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), transform_frame_, mounting_frame_));
}

ROSCameraHousingBroadcastTInterface::ROSCameraHousingBroadcastTInterface(const string& transform_frame,
									 const string& mounting_frame,
									 const string& housing_frame,
                                                                         const Pose6d& pose) :
  ROSCameraTransformInterface( transform_frame, mounting_frame, housing_frame)
{
  pose_ = pose;
  transform_.child_frame_id_ = transform_frame_;
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
}

bool ROSCameraHousingBroadcastTInterface::pushTransform(Pose6d& pose)
{
  pose_ = pose;			// store the pose from transform_frame to mounting_frame
  return (true);
}

bool ROSCameraHousingBroadcastTInterface::store(string& filePath)
{
  std::ofstream outputFile(filePath.c_str(), std::ios::app);  // open for appending
  if (outputFile.is_open())
  {
    // Camer optical frame to ref is estimated by bundle adjustment  optical2ref
    // Camer housing to camera optical frame is specified by urdf   optical2housing
    // Desired ref2optical = optical2ref^-1 * optical2housing


    // compute pose from mounting to transform_frame
    Pose6d mounting2transform = pose_.getInverse();
    Pose6d transform2housing  = getPoseFromTF(transform_frame_, housing_frame_, tf_listener_);
    Pose6d mounting2housing  = mounting2transform * transform2housing;
    
    if (!ref_frame_initialized_)
      {
	return (false);  // timer won't start publishing until ref_frame_ is defined
      }


    
    // append the transform to a launch file
    double qx, qy, qz, qw;
    mounting2housing.getQuaternion(qx, qy, qz, qw);
    outputFile << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"";
    outputFile << transform_frame_ << "_tf_broadcaster"
               << "\" args=\"";
    outputFile << mounting2housing.x << ' ' << mounting2housing.y << ' ' << mounting2housing.z << ' ';
    outputFile << qx << ' ' << qy << ' ' << qz << ' ' << qw;
    outputFile << " " << mounting_frame_;
    outputFile << " " << housing_frame_;
    outputFile << " 100\" />" << std::endl;
    outputFile.close();
    return (true);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file:" << filePath);
    return false;
  }  // end if writing to file
}

void ROSCameraHousingBroadcastTInterface::setReferenceFrame(string& ref_frame)
{
  static ros::NodeHandle nh;
  ref_frame_ = ref_frame;
  ref_frame_initialized_ = true;
  timer_ = nh.createTimer(ros::Rate(1.0), &ROSCameraHousingBroadcastTInterface::timerCallback, this);
}

void ROSCameraHousingBroadcastTInterface::timerCallback(const ros::TimerEvent& timer_event)
{  // broadcast current value of pose.inverse() as a transform each time called

  // compute pose from mounting to transform_frame
  Pose6d mounting2transform = pose_.getInverse();
  Pose6d transform2housing  = getPoseFromTF(transform_frame_, housing_frame_, tf_listener_);
  Pose6d mounting2housing  = mounting2transform * transform2housing;


  transform_.setBasis(mounting2housing.getBasis());
  transform_.setOrigin(mounting2housing.getOrigin());
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), mounting_frame_, housing_frame_));
}

Pose6d ROSCameraHousingBroadcastTInterface::pullTransform()
{
  return (pose_);
}

ROSCameraHousingCalTInterface::ROSCameraHousingCalTInterface(const string& transform_frame,
							     const string& mounting_frame,
							     const string& housing_frame):
  ROSCameraTransformInterface(transform_frame, mounting_frame, housing_frame)
{
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
  nh_ = new ros::NodeHandle;

  get_client_ = nh_->serviceClient<industrial_extrinsic_cal::get_mutable_joint_states>("get_mutable_joint_states");
  set_client_ = nh_->serviceClient<industrial_extrinsic_cal::set_mutable_joint_states>("set_mutable_joint_states");
  store_client_ =
      nh_->serviceClient<industrial_extrinsic_cal::store_mutable_joint_states>("store_mutable_joint_states");

  get_request_.joint_names.push_back(housing_frame + "_x_joint");
  get_request_.joint_names.push_back(housing_frame + "_y_joint");
  get_request_.joint_names.push_back(housing_frame + "_z_joint");
  get_request_.joint_names.push_back(housing_frame + "_yaw_joint");
  get_request_.joint_names.push_back(housing_frame + "_pitch_joint");
  get_request_.joint_names.push_back(housing_frame + "_roll_joint");

  set_request_.joint_names.push_back(housing_frame + "_x_joint");
  set_request_.joint_names.push_back(housing_frame + "_y_joint");
  set_request_.joint_names.push_back(housing_frame + "_z_joint");
  set_request_.joint_names.push_back(housing_frame + "_yaw_joint");
  set_request_.joint_names.push_back(housing_frame + "_pitch_joint");
  set_request_.joint_names.push_back(housing_frame + "_roll_joint");

  while (!get_client_.call(get_request_, get_response_))
  {
    sleep(1);
    ROS_INFO("Waiting for mutable joint state publisher to come up");
  }
  for (int i = 0; i < (int)get_response_.joint_values.size(); i++)
  {
    joint_values_.push_back(get_response_.joint_values[i]);
  }
}

Pose6d ROSCameraHousingCalTInterface::pullTransform()
{
  // mounting to housing is maintained by mutable joint state publisher, we want from transform_frame to mounting frame_
  pose_ = getPoseFromTF(transform_frame_, mounting_frame_, tf_listener_);
  return (pose_);
}

bool ROSCameraHousingCalTInterface::pushTransform(Pose6d& pose)
{
  // pushed transform is from transform_frame to mounting_frame
  Pose6d pose_inverse = pose.getInverse();

  // get transform from optical frame to housing frame from tf
  Pose6d optical2housing = getPoseFromTF(transform_frame_, housing_frame_, tf_listener_);

  // compute the desired transform
  Pose6d mount2housing = pose_inverse * optical2housing;

  // convert to xyz, roll, pitch and yaw for client
  double ez, ey, ex;
  mount2housing.getEulerZYX(ez, ey, ex);

  set_request_.joint_values.clear();
  set_request_.joint_values.push_back(mount2housing.x);
  set_request_.joint_values.push_back(mount2housing.y);
  set_request_.joint_values.push_back(mount2housing.z);
  set_request_.joint_values.push_back(ez);
  set_request_.joint_values.push_back(ey);
  set_request_.joint_values.push_back(ex);
  set_client_.call(set_request_, set_response_);
  return (true);
}

bool ROSCameraHousingCalTInterface::store(string& filePath)
{
  // NOTE, file_name is not used, but is kept here for consistency with store functions of other transform interfaces
  store_client_.call(store_request_, store_response_);
  return (true);
}

void ROSCameraHousingCalTInterface::setReferenceFrame(string& ref_frame)
{
  ref_frame_ = ref_frame;
  ref_frame_initialized_ = true;
}

ROSSimpleCalTInterface::ROSSimpleCalTInterface(const string& transform_frame, const string& parent_frame)
{
  transform_frame_ = transform_frame;
  parent_frame_ = parent_frame;
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
  nh_ = new ros::NodeHandle;

  get_client_ = nh_->serviceClient<industrial_extrinsic_cal::get_mutable_joint_states>("get_mutable_joint_states");
  set_client_ = nh_->serviceClient<industrial_extrinsic_cal::set_mutable_joint_states>("set_mutable_joint_states");
  store_client_ =
      nh_->serviceClient<industrial_extrinsic_cal::store_mutable_joint_states>("store_mutable_joint_states");

  get_request_.joint_names.push_back(transform_frame + "_x_joint");
  get_request_.joint_names.push_back(transform_frame + "_y_joint");
  get_request_.joint_names.push_back(transform_frame + "_z_joint");
  get_request_.joint_names.push_back(transform_frame + "_yaw_joint");
  get_request_.joint_names.push_back(transform_frame + "_pitch_joint");
  get_request_.joint_names.push_back(transform_frame + "_roll_joint");

  set_request_.joint_names.push_back(transform_frame + "_x_joint");
  set_request_.joint_names.push_back(transform_frame + "_y_joint");
  set_request_.joint_names.push_back(transform_frame + "_z_joint");
  set_request_.joint_names.push_back(transform_frame + "_yaw_joint");
  set_request_.joint_names.push_back(transform_frame + "_pitch_joint");
  set_request_.joint_names.push_back(transform_frame + "_roll_joint");

  if (get_client_.call(get_request_, get_response_))
  {
    for (int i = 0; i < (int)get_response_.joint_values.size(); i++)
    {
      joint_values_.push_back(get_response_.joint_values[i]);
    }
  }
  else
  {
    ROS_ERROR("ros_scti get_client_ returned false, looking for %s", transform_frame_.c_str());
  }
}

Pose6d ROSSimpleCalTInterface::pullTransform()
{
  // The computed transform from the reference frame to the optical frame is composed of 3 transforms
  // one from reference frame to mounting frame
  // one composed of the 6DOF unknowns we are trying to calibrate
  // from the mounting frame to the housing. It's values are maintained by the mutable joint state publisher
  // the third is from housing to optical frame

  get_client_.call(get_request_, get_response_);
  pose_.setOrigin(get_response_.joint_values[0], get_response_.joint_values[1], get_response_.joint_values[2]);
  pose_.setEulerZYX(get_response_.joint_values[3], get_response_.joint_values[4], get_response_.joint_values[5]);
  return (pose_);
}

bool ROSSimpleCalTInterface::pushTransform(Pose6d& pose)
{
  double ez, ey, ex;
  pose.getEulerZYX(ez, ey, ex);

  set_request_.joint_values.clear();
  set_request_.joint_values.push_back(pose.x);
  set_request_.joint_values.push_back(pose.y);
  set_request_.joint_values.push_back(pose.z);
  set_request_.joint_values.push_back(ez);
  set_request_.joint_values.push_back(ey);
  set_request_.joint_values.push_back(ex);
  set_client_.call(set_request_, set_response_);

  return (true);
}

bool ROSSimpleCalTInterface::store(string& filePath)
{
  // NOTE, file_name is not used, but is kept here for consistency with store functions of other transform interfaces
  store_client_.call(store_request_, store_response_);
  return (true);
}

void ROSSimpleCalTInterface::setReferenceFrame(string& ref_frame)
{
  ref_frame_ = ref_frame;
  ref_frame_initialized_ = true;
}

Pose6d ROSSimpleCalTInterface::getIntermediateFrame()
{
  Pose6d pose(0, 0, 0, 0, 0, 0);
  if (ref_frame_initialized_)
  {
    pose = getPoseFromTF(ref_frame_, parent_frame_, tf_listener_);
  }
  else
  {
    ROS_ERROR("ROSSimpleCalTInterface requires ref_frame_ initialized before calling getIntermediateFrame()");
  }
  return (pose);
}

ROSSimpleCameraCalTInterface::ROSSimpleCameraCalTInterface(const string& transform_frame, const string& camera_mounting_frame):
  ROSCameraTransformInterface(transform_frame, camera_mounting_frame, "")
{
  ref_frame_initialized_ = false;  // still need to initialize ref_frame_
  nh_ = new ros::NodeHandle;

  get_client_ = nh_->serviceClient<industrial_extrinsic_cal::get_mutable_joint_states>("get_mutable_joint_states");
  set_client_ = nh_->serviceClient<industrial_extrinsic_cal::set_mutable_joint_states>("set_mutable_joint_states");
  store_client_ =
      nh_->serviceClient<industrial_extrinsic_cal::store_mutable_joint_states>("store_mutable_joint_states");

  get_request_.joint_names.push_back(transform_frame + "_x_joint");
  get_request_.joint_names.push_back(transform_frame + "_y_joint");
  get_request_.joint_names.push_back(transform_frame + "_z_joint");
  get_request_.joint_names.push_back(transform_frame + "_yaw_joint");
  get_request_.joint_names.push_back(transform_frame + "_pitch_joint");
  get_request_.joint_names.push_back(transform_frame + "_roll_joint");
  set_request_.joint_names.push_back(transform_frame + "_x_joint");
  set_request_.joint_names.push_back(transform_frame + "_y_joint");
  set_request_.joint_names.push_back(transform_frame + "_z_joint");
  set_request_.joint_names.push_back(transform_frame + "_yaw_joint");
  set_request_.joint_names.push_back(transform_frame + "_pitch_joint");
  set_request_.joint_names.push_back(transform_frame + "_roll_joint");

  if (get_client_.call(get_request_, get_response_))
  {
    for (int i = 0; i < (int)get_response_.joint_values.size(); i++)
    {
      joint_values_.push_back(get_response_.joint_values[i]);
    }
  }
  else
  {
    ROS_ERROR("get_client_ returned false");
  }
}

Pose6d ROSSimpleCameraCalTInterface::pullTransform()
{
  // pose from mounting_frame to transform_frame is maintained by mutable joint state publisher and tf
  pose_ = getPoseFromTF(transform_frame_, mounting_frame_, tf_listener_);
  return (pose_);
}

bool ROSSimpleCameraCalTInterface::pushTransform(Pose6d& pose)
{
  double ez, ey, ex;
  pose_ = pose;
  Pose6d ipose = pose_.getInverse();
  ipose.getEulerZYX(ez, ey, ex);

  set_request_.joint_values.clear();
  set_request_.joint_values.push_back(ipose.x);
  set_request_.joint_values.push_back(ipose.y);
  set_request_.joint_values.push_back(ipose.z);
  set_request_.joint_values.push_back(ez);
  set_request_.joint_values.push_back(ey);
  set_request_.joint_values.push_back(ex);
  set_client_.call(set_request_, set_response_);

  return (true);
}

bool ROSSimpleCameraCalTInterface::store(string& filePath)
{
  // NOTE, file_name is not used, but is kept here for consistency with store functions of other transform interfaces
  store_client_.call(store_request_, store_response_);
  return (true);
}

void ROSSimpleCameraCalTInterface::setReferenceFrame(string& ref_frame)
{
  ref_frame_ = ref_frame;
  ref_frame_initialized_ = true;
}

}  // end namespace industrial_extrinsic_cal
