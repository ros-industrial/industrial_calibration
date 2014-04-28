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

  ROSListenerTransInterface::ROSListenerTransInterface(const string transform_frame) 
  {
    transform_frame_ = transform_frame;
    transform_.child_frame_id_ = transform_frame_;
    ref_frame_initialized_ = false;    // still need to initialize ref_frame_
  }				

  Pose6d  ROSListenerTransInterface::pullTransform()
  {
    if(!ref_frame_initialized_){
      Pose6d pose(0,0,0,0,0,0);
      ROS_ERROR("Trying to pull transform from interface without setting reference frame");
      return(pose);
    }
    else{
      tf::StampedTransform transform;
      ros::Time now = ros::Time::now();
      while(! tf_listener_.waitForTransform(transform_frame_,ref_frame_, now, ros::Duration(1.0))){
	ROS_ERROR("waiting for tranform: %s to reference: %s",transform_frame_.c_str(),ref_frame_.c_str());
      }
      tf_listener_.lookupTransform(transform_frame_,ref_frame_, now, transform);
      pose_.setBasis(transform.getBasis());
      pose_.setOrigin(transform.getOrigin());
      return(pose_);
    }
  }

  ROSCameraListenerTransInterface::ROSCameraListenerTransInterface(const string transform_frame) 
  {
    transform_frame_ = transform_frame;
    transform_.child_frame_id_ = transform_frame_;
    ref_frame_initialized_ = false;    // still need to initialize ref_frame_
  }				

  Pose6d  ROSCameraListenerTransInterface::pullTransform()
  {
    if(!ref_frame_initialized_){
      Pose6d pose(0,0,0,0,0,0);
      ROS_ERROR("Trying to pull transform from interface without setting reference frame");
      return(pose);
    }
    else{
      tf::StampedTransform transform;
      ros::Time now = ros::Time::now();
      while(! tf_listener_.waitForTransform(ref_frame_,transform_frame_, now, ros::Duration(1.0))){
	ROS_ERROR("waiting for tranform: %s to reference: %s",transform_frame_.c_str(),ref_frame_.c_str());
      }
      tf_listener_.lookupTransform(ref_frame_,transform_frame_, now, transform);
      pose_.setBasis(transform.getBasis());
      pose_.setOrigin(transform.getOrigin());
      return(pose_);
    }
  }

  /** @brief this object is intened to be used for cameras not targets
   *            It simply listens to a pose from camera's optical frame to reference frame, this must be set in a urdf
   *            This is the inverse of the transform from world to camera's optical frame
   *            push does nothing
   *            store does nothing
   */
  ROSCameraHousingListenerTInterface::ROSCameraHousingListenerTInterface(const string transform_frame, const string housing_frame) 
  {
    transform_frame_               = transform_frame;
    housing_frame_                  = housing_frame; // note, this is not used, but maintained to be symetric with Broadcaster parameter list
    transform_.child_frame_id_ = transform_frame_;
    ref_frame_initialized_         = false;    // still need to initialize ref_frame_
  }				

  Pose6d  ROSCameraHousingListenerTInterface::pullTransform()
  {
    if(!ref_frame_initialized_){
      Pose6d pose(0,0,0,0,0,0);
      ROS_ERROR("Trying to pull transform from interface without setting reference frame");
      return(pose);
    }
    else{
      tf::StampedTransform transform;
      ros::Time now = ros::Time::now();
      while(! tf_listener_.waitForTransform(ref_frame_,transform_frame_, now, ros::Duration(1.0))){
	ROS_ERROR("waiting for tranform: %s to reference: %s",transform_frame_.c_str(),ref_frame_.c_str());
      }
      tf_listener_.lookupTransform(ref_frame_,transform_frame_, now, transform);
      pose_.setBasis(transform.getBasis());
      pose_.setOrigin(transform.getOrigin());
      return(pose_);
    }
  }

  ROSBroadcastTransInterface::ROSBroadcastTransInterface(const string transform_frame, Pose6d pose)
  {
    transform_frame_                = transform_frame;
    transform_.child_frame_id_ = transform_frame_;
    ref_frame_initialized_         = false;    // still need to initialize ref_frame_
    pose_                                 = pose;
  }

  bool   ROSBroadcastTransInterface::pushTransform(Pose6d & pose)
  {
    pose_ = pose; 
    if(!ref_frame_initialized_){ 
      return(false);		// timer won't start publishing until ref_frame_ is defined
    }
    return(true);
  }

  bool  ROSBroadcastTransInterface::store(std::string filePath)
  {
    std::ofstream outputFile(filePath.c_str(), std::ios::app); // open for appending
    if (outputFile.is_open())
      {
	double qx,qy,qz,qw;
	pose_.getQuaternion(qx, qy, qz, qw);
	outputFile<<"<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"";
	outputFile<< transform_frame_ <<"_tf_broadcaster"<<"\" args=\"";
	outputFile<< pose_.x << ' '<< pose_.y << ' '<< pose_.z << ' ';
	outputFile<< qx << ' '<< qy << ' '<<qz << ' ' << qw ;
	outputFile<<" "<< ref_frame_;
	outputFile<<" "<<transform_frame_;
	outputFile<<" 100\" />"<<std::endl;
	outputFile.close();	
	return(true);
      }
    else
      {
	ROS_ERROR_STREAM("Unable to open file:" <<filePath);
	return false;
      }//end if writing to file
  }

  void  ROSBroadcastTransInterface::setReferenceFrame(string ref_frame)
  {
    static ros::NodeHandle nh;
    ref_frame_              = ref_frame;
    ref_frame_defined_ = true;
    timer_                     = nh.createTimer(ros::Rate(1.0),&ROSBroadcastTransInterface::timerCallback, this);
  }

  void  ROSBroadcastTransInterface::timerCallback(const ros::TimerEvent & timer_event)
  { // broadcast current value of pose as a transform each time called
    transform_.setBasis(pose_.getBasis());
    transform_.setOrigin(pose_.getOrigin());
    transform_.child_frame_id_ = transform_frame_;
    transform_.frame_id_ = ref_frame_;
    //    ROS_INFO("broadcasting %s in %s",transform_frame_.c_str(),ref_frame_.c_str());
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), transform_frame_, ref_frame_));
  }

  ROSCameraBroadcastTransInterface::ROSCameraBroadcastTransInterface(const string transform_frame, Pose6d pose)
  {
    transform_frame_                = transform_frame;
    transform_.child_frame_id_ = transform_frame_;
    ref_frame_initialized_         = false;    // still need to initialize ref_frame_
    pose_                                 = pose;
  }

  bool   ROSCameraBroadcastTransInterface::pushTransform(Pose6d & pose)
  {
    pose_ = pose; 
    if(!ref_frame_initialized_){ 
      return(false);		// timer won't start publishing until ref_frame_ is defined
    }
    return(true);
  }

  bool  ROSCameraBroadcastTransInterface::store(std::string filePath)
  {
    std::ofstream outputFile(filePath.c_str(), std::ios::app); // open for appending
    if (outputFile.is_open())
      {
	double qx,qy,qz,qw;
	pose_.getInverse().getQuaternion(qx, qy, qz, qw);
	outputFile<<"<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"";
	outputFile<<transform_frame_<<"_tf_broadcaster"<<"\" args=\"";
	outputFile<< pose_.getInverse().x << ' '<< pose_.getInverse().y << ' '<< pose_.getInverse().z << ' ';
	outputFile<< qx << ' '<< qy << ' '<<qz << ' ' << qw ;
	outputFile<<" "<<ref_frame_;
	outputFile<<" "<<transform_frame_;
	outputFile<<" 100\" />"<<std::endl;
	outputFile.close();	
	return(true);
      }
    else
      {
	ROS_ERROR_STREAM("Unable to open file:" <<filePath);
	return false;
      }//end if writing to file
  }

  void  ROSCameraBroadcastTransInterface::setReferenceFrame(string ref_frame)
  {
    static ros::NodeHandle nh;
    ref_frame_              = ref_frame;
    ref_frame_defined_ = true;
    timer_                     = nh.createTimer(ros::Rate(1.0),&ROSCameraBroadcastTransInterface::timerCallback, this);
  }

  void  ROSCameraBroadcastTransInterface::timerCallback(const ros::TimerEvent & timer_event)
  { // broadcast current value of pose.inverse() as a transform each time called
    transform_.setBasis(pose_.getInverse().getBasis());
    transform_.setOrigin(pose_.getInverse().getOrigin());
    transform_.child_frame_id_ = transform_frame_;
    transform_.frame_id_ = ref_frame_;
    //    ROS_INFO("broadcasting %s in %s",transform_frame_.c_str(),ref_frame_.c_str());
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), transform_frame_, ref_frame_));
  }

  ROSCameraHousingBroadcastTInterface::ROSCameraHousingBroadcastTInterface(const string transform_frame, Pose6d pose)
  {
    transform_frame_                = transform_frame;
    transform_.child_frame_id_ = transform_frame_;
    ref_frame_initialized_         = false;    // still need to initialize ref_frame_
    pose_                                 = pose;
  }

  bool   ROSCameraHousingBroadcastTInterface::pushTransform(Pose6d & pose)
  {
    pose_ = pose; 
    if(!ref_frame_initialized_){ 
      return(false);		// timer won't start publishing until ref_frame_ is defined
    }
    return(true);
  }

  bool  ROSCameraHousingBroadcastTInterface::store(std::string filePath)
  {
    std::ofstream outputFile(filePath.c_str(), std::ios::app); // open for appending
    if (outputFile.is_open()){
      // Camer optical frame to ref is estimated by bundle adjustment  T_co2ref
      // Camer housing to camera optical frame is specified by urdf   T_ch2co
      // Desired T_ref2ch = T_co2ref^(-1) * T_ch2co^(-1)
      // To get T_ch2co^(-1) we may use the tf listener
      
      Pose6d T_co2ch;
      tf::StampedTransform transform;
      ros::Time now = ros::Time::now();
      while(! tf_listener_.waitForTransform(transform_frame_,housing_frame_, now, ros::Duration(1.0))){
	ROS_ERROR("waiting for tranform: %s to reference: %s",transform_frame_.c_str(),housing_frame_.c_str());
      }
      tf_listener_.lookupTransform(transform_frame_, housing_frame_, now, transform);
      T_co2ch.setBasis(transform.getBasis());
      T_co2ch.setOrigin(transform.getOrigin());
      Pose6d T_ref2ch = pose_.getInverse() * T_co2ch;
      
      // append the transform to a launch file
      double qx,qy,qz,qw;
      T_ref2ch.getQuaternion(qx, qy, qz, qw);
      outputFile<<"<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"";
      outputFile<<transform_frame_<<"_tf_broadcaster"<<"\" args=\"";
      outputFile<< T_ref2ch.x << ' '<< T_ref2ch.y << ' '<< T_ref2ch.z << ' ';
      outputFile<< qx << ' '<< qy << ' '<<qz << ' ' << qw ;
      outputFile<<" "<<ref_frame_;
      outputFile<<" "<<transform_frame_;
      outputFile<<" 100\" />"<<std::endl;
      outputFile.close();	
      return(true);
    }
    else{
      ROS_ERROR_STREAM("Unable to open file:" <<filePath);
      return false;
    }//end if writing to file
  }

  void  ROSCameraHousingBroadcastTInterface::setReferenceFrame(string ref_frame)
  {
    static ros::NodeHandle nh;
    ref_frame_              = ref_frame;
    ref_frame_defined_ = true;
    timer_                     = nh.createTimer(ros::Rate(1.0),&ROSCameraHousingBroadcastTInterface::timerCallback, this);
  }

  void  ROSCameraHousingBroadcastTInterface::timerCallback(const ros::TimerEvent & timer_event)
  { // broadcast current value of pose.inverse() as a transform each time called

    // Camer optical frame to ref is estimated by bundle adjustment  T_co2ref
    // Camer housing to camera optical frame is specified by urdf   T_ch2co
    // Desired T_ref2ch = T_co2ref^(-1) * T_ch2co^(-1)
    // To get T_ch2co^(-1) we may use the tf listener

    Pose6d T_co2ch;
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    while(! tf_listener_.waitForTransform(transform_frame_,housing_frame_, now, ros::Duration(1.0))){
      ROS_ERROR("waiting for tranform: %s to reference: %s",transform_frame_.c_str(),housing_frame_.c_str());
    }
    tf_listener_.lookupTransform(transform_frame_, housing_frame_, now, transform);
    T_co2ch.setBasis(transform.getBasis());
    T_co2ch.setOrigin(transform.getOrigin());
    Pose6d T_ref2ch = pose_.getInverse() * T_co2ch;
    
    // copy into the stamped transform
    transform_.setBasis(T_ref2ch.getBasis());
    transform_.setOrigin(T_ref2ch.getOrigin());
    transform_.child_frame_id_ = housing_frame_;
    transform_.frame_id_ = ref_frame_;
    //    ROS_INFO("broadcasting %s in %s",housing_frame_.c_str(),ref_frame_.c_str());
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), housing_frame_, ref_frame_));
  }
} // end namespace industrial_extrinsic_cal
