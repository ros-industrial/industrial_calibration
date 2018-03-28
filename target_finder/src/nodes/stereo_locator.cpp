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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h> 
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp> 
#include <target_finder/stereo_locator.h> // service call definintion
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

using std::string;
using boost::shared_ptr;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Camera;
using industrial_extrinsic_cal::CameraParameters;
using industrial_extrinsic_cal::NoWaitTrigger;

class StereoLocatorService 
{
public:
  StereoLocatorService(ros::NodeHandle nh);
  ~StereoLocatorService()  {  } ;
  bool getPoseCallBack( target_finder::stereo_locator::Request &req, target_finder::stereo_locator::Response &res);
  void initMCircleTarget(int rows, int cols, double circle_dia, double spacing);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer rail_cal_server_;
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;
  shared_ptr<Target> target_;
  shared_ptr<Camera> left_camera_;
  shared_ptr<Camera> right_camera_;
  double focal_length_x_;
  double focal_length_y_;
  bool   using_rectified_images_;
  string left_image_topic_;
  string left_camera_name_;
  string left_camera_optical_frame_;
  string right_image_topic_;
  string right_camera_name_;
  string right_camera_optical_frame_;
  int target_rows_;
  int target_cols_;
  double circle_spacing_;
  double circle_diameter_;
  int left_image_height_;
  int left_image_width_;
  int right_image_height_;
  int right_image_width_;
  Roi left_roi_;
  Roi right_roi_;
  double D0_;
  double qx_, qy_, qz_, qw_;
  CameraParameters left_camera_parameters_;
  CameraParameters right_camera_parameters_;
  double allowable_cost_per_observation_;
  double lfx_,lfy_,lcx_,lcy_;
  double lk1_,lk2_,lk3_,lp1_,lp2_;
  double rfx_,rfy_,rcx_,rcy_;
  double rk1_,rk2_,rk3_,rp1_,rp2_;
  tf::TransformListener listener_;
};

StereoLocatorService::StereoLocatorService(ros::NodeHandle nh)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");
  allowable_cost_per_observation_ = 1.0;

  if(!pnh.getParam( "using_rectified_images", using_rectified_images_)){
    using_rectified_images_ = false;
    ROS_INFO("Assuming images are raw, not rectified");
  }

  if(!pnh.getParam( "left_image_topic", left_image_topic_)){
    ROS_ERROR("Must set param:  left_image_topic");
  }

  if(!pnh.getParam( "left_camera_name", left_camera_name_)){
    ROS_ERROR("Must set param:  left_camera_name");
  }

  if(!pnh.getParam( "left_camera_optical_frame", left_camera_optical_frame_)){
    ROS_ERROR("Must set param:  left_camera_optical_frame");
  }

  if(!pnh.getParam( "right_image_topic", right_image_topic_)){
    ROS_ERROR("Must set param:  right_image_topic");
  }

  if(!pnh.getParam( "right_camera_name", right_camera_name_)){
    ROS_ERROR("Must set param:  right_camera_name");
  }

  if(!pnh.getParam( "right_camera_optical_frame", right_camera_optical_frame_)){
    ROS_ERROR("Must set param:  right_camera_optical_frame");
  }

  if(!pnh.getParam( "target_rows", target_rows_)){
    ROS_ERROR("Must set param:  target_rows");
  }

  if(!pnh.getParam( "target_cols", target_cols_)){
    ROS_ERROR("Must set param:  target_cols");
  }

  if(!pnh.getParam( "target_circle_dia", circle_diameter_)){
    ROS_ERROR("Must set param:  target_circle_dia");
  }

  if(!pnh.getParam( "target_spacing", circle_spacing_)){
    ROS_ERROR("Must set param:  target_spacing");
  }

  initMCircleTarget(target_rows_, target_cols_, circle_diameter_, circle_spacing_);

  
  bool use_quaternion = false;
  if(pnh.getParam( "qx", qx_))
  {
    if(pnh.getParam( "qy", qy_))
    {
      if(pnh.getParam( "qz", qz_))
      {
        if(pnh.getParam( "qw", qw_))
        {
          use_quaternion = true;
        }
      }
    }
  }
  if(!pnh.getParam( "camera_to_target_distance", D0_)){
    ROS_INFO("Did not set param camera_to_target_distance to approximate value, using 1.0m");
    D0_ = 1.0;
  }


  Pose6d Ptemp;
  Ptemp.setOrigin(-0.1, -0.1, D0_);
  if(!use_quaternion)
  {
    tf::Matrix3x3 m;
    m[0][0] = 1; m[0][1] =   0; m[0][2] = 0;
    m[1][0] = 0; m[1][1] = -1.; m[1][2] = 0;
    m[2][0] = 0; m[2][1] =   0; m[2][2] = -1;
    Ptemp.setBasis(m);
    Ptemp.getQuaternion(qx_, qy_, qz_, qw_);
  }
  else{
    Ptemp.setQuaternion(qx_, qy_, qz_, qw_);
  }
  Ptemp.show("initial pose");
  ROS_WARN("parameters qx, qy, qz, and qw not provided, using default values of (%.2f, %.2f, %.2f, %.2f)", qx_, qy_, qz_, qw_);
  target_->pose_ = Ptemp;

  bool is_moving = true;
  left_camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera("left_camera", left_camera_parameters_, is_moving));
  right_camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera("right_camera", right_camera_parameters_, is_moving));
  left_camera_->trigger_ = shared_ptr<NoWaitTrigger>(new NoWaitTrigger());
  right_camera_->trigger_ = shared_ptr<NoWaitTrigger>(new NoWaitTrigger());
  left_camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(left_image_topic_, left_camera_name_));
  right_camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(right_image_topic_, right_camera_name_));

  sleep(10); // wait for camera to come up or else pulCameraInfo will fail ungracefully
  if(!left_camera_->camera_observer_->pullCameraInfo(left_camera_->camera_parameters_.focal_length_x,
                                           left_camera_->camera_parameters_.focal_length_y,
                                           left_camera_->camera_parameters_.center_x,
                                           left_camera_->camera_parameters_.center_y,
                                           left_camera_->camera_parameters_.distortion_k1,
                                           left_camera_->camera_parameters_.distortion_k2,
                                           left_camera_->camera_parameters_.distortion_k3,
                                           left_camera_->camera_parameters_.distortion_p1,
                                           left_camera_->camera_parameters_.distortion_p2,
                                           left_image_width_, left_image_height_))
  {
    ROS_FATAL("Could not get camera information for %s from topic %s. Shutting down node.", left_camera_name_.c_str(), left_image_topic_.c_str());
    ros::shutdown();
  }

  if(!right_camera_->camera_observer_->pullCameraInfo(right_camera_->camera_parameters_.focal_length_x,
                                           right_camera_->camera_parameters_.focal_length_y,
                                           right_camera_->camera_parameters_.center_x,
                                           right_camera_->camera_parameters_.center_y,
                                           right_camera_->camera_parameters_.distortion_k1,
                                           right_camera_->camera_parameters_.distortion_k2,
                                           right_camera_->camera_parameters_.distortion_k3,
                                           right_camera_->camera_parameters_.distortion_p1,
                                           right_camera_->camera_parameters_.distortion_p2,
                                           right_image_width_, right_image_height_))
  {
    ROS_FATAL("Could not get camera information for %s from topic %s. Shutting down node.", right_camera_name_.c_str(), right_image_topic_.c_str());
    ros::shutdown();
  }

  lfx_ = left_camera_->camera_parameters_.focal_length_x;
  lfy_ = left_camera_->camera_parameters_.focal_length_y;
  lcx_ = left_camera_->camera_parameters_.center_x;
  lcy_ = left_camera_->camera_parameters_.center_y;
  rfx_ = right_camera_->camera_parameters_.focal_length_x;
  rfy_ = right_camera_->camera_parameters_.focal_length_y;
  rcx_ = right_camera_->camera_parameters_.center_x;
  rcy_ = right_camera_->camera_parameters_.center_y;
  if(using_rectified_images_){
    lk1_ = 0.0;
    lk2_ = 0.0;
    lk3_ = 0.0;
    lp1_ = 0.0;
    lp2_ = 0.0;
    rk1_ = 0.0;
    rk2_ = 0.0;
    rk3_ = 0.0;
    rp1_ = 0.0;
    rp2_ = 0.0;
  }
  else{
    lk1_ = left_camera_->camera_parameters_.distortion_k1;
    lk2_ = left_camera_->camera_parameters_.distortion_k2;
    lk3_ = left_camera_->camera_parameters_.distortion_k3;
    lp1_ = left_camera_->camera_parameters_.distortion_p1;
    lp2_ = left_camera_->camera_parameters_.distortion_p2;
    rk1_ = right_camera_->camera_parameters_.distortion_k1;
    rk2_ = right_camera_->camera_parameters_.distortion_k2;
    rk3_ = right_camera_->camera_parameters_.distortion_k3;
    rp1_ = right_camera_->camera_parameters_.distortion_p1;
    rp2_ = right_camera_->camera_parameters_.distortion_p2;
  }

  // set ROI's to include whole image for both cameras
  left_roi_.x_min = 0;
  left_roi_.y_min = 0;
  left_roi_.x_max = left_image_width_;
  left_roi_.y_max = left_image_height_;
  right_roi_.x_min = 0;
  right_roi_.y_min = 0;
  right_roi_.x_max = right_image_width_;
  right_roi_.y_max = right_image_height_;

  ROS_INFO("left camera info focal:%f %f center:%f %f  radial:%f %f %f tang: %f %f",
	   lfx_,
	   lfy_,
	   lcx_,
	   lcy_,
	   lk1_,
	   lk2_,
	   lk3_,
	   lp1_,
	   lp2_
	   );

  ROS_INFO("right camera info focal:%f %f center:%f %f  radial:%f %f %f tang: %f %f",
	   rfx_,
	   rfy_,
	   rcx_,
	   rcy_,
	   rk1_,
	   rk2_,
	   rk3_,
	   rp1_,
	   rp2_
	   );

  // call this service to find out where the target is with respect to the left camera frame
  rail_cal_server_ = nh_.advertiseService( "StereoLocatorService", &StereoLocatorService::getPoseCallBack, this);
  
}

// moves the stage through a series of positions and solves for the tranform between the two cameras
bool StereoLocatorService::getPoseCallBack( target_finder::stereo_locator::Request &req, target_finder::stereo_locator::Response &res)
{
  allowable_cost_per_observation_ = req.allowable_cost_per_observation;
  ros::NodeHandle nh;
  CameraObservations left_camera_observations;
  CameraObservations right_camera_observations;
  Problem problem;
  CostFunction* cost_function[target_->num_points_];

  industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::StereoTargetLocator;
  tf::StampedTransform transform;
  try{
    listener_.lookupTransform(left_camera_optical_frame_, right_camera_optical_frame_, ros::Time(0), transform);
			      
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return(0);
  }
  // copy the transform into a Pos6D
  Pose6d LCtoRC;
  LCtoRC.setOrigin(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  LCtoRC.setBasis(transform.getBasis());
  LCtoRC.show("LCtoRC");
  
  left_camera_->camera_observer_->clearObservations();
  right_camera_->camera_observer_->clearObservations();

  left_camera_->camera_observer_->clearTargets();
  right_camera_->camera_observer_->clearTargets();

  left_camera_->camera_observer_->addTarget(target_, left_roi_, cost_type);
  right_camera_->camera_observer_->addTarget(target_, right_roi_, cost_type);

  left_camera_->camera_observer_->triggerCamera();
  right_camera_->camera_observer_->triggerCamera();
  

  // wait until the observations are done
  while (!left_camera_->camera_observer_->observationsDone() && !right_camera_->camera_observer_->observationsDone()) ;
  bool left_ok  = left_camera_->camera_observer_->getObservations(left_camera_observations);
  bool right_ok = right_camera_->camera_observer_->getObservations(right_camera_observations);
  if(left_ok && right_ok){
    ROS_INFO("Found %d observations in left image",(int) left_camera_observations.size());
    ROS_INFO("Found %d observations in right image",(int) right_camera_observations.size());
    int nleft = (int) left_camera_observations.size();
    int nright = (int) right_camera_observations.size();
    if(nleft != target_->num_points_ || nright != target_->num_points_ ){
      ROS_ERROR("Could not find target in image num_left_obs = %d num_rt_obs = %d", nleft, nright);
    }
    else{	// add a new cost to the problem for each observation
      for(int j=0; j<target_->num_points_; j++){
	double left_image_x  = left_camera_observations[j].image_loc_x;
	double left_image_y  = left_camera_observations[j].image_loc_y;
	Point3d left_point   = target_->pts_[left_camera_observations[j].point_id]; // don't assume ordering from camera observer
	double right_image_x = right_camera_observations[j].image_loc_x;
	double right_image_y = right_camera_observations[j].image_loc_y;
	Point3d right_point  = target_->pts_[right_camera_observations[j].point_id]; // don't assume ordering from camera observer
	cost_function[j] = industrial_extrinsic_cal::StereoTargetLocator::Create(left_image_x, left_image_y, left_point,
										right_image_x, right_image_y, right_point,
										LCtoRC,
										lfx_, lfy_,
										lcx_, lcy_,
										lk1_, lk2_, lk3_,
										lp1_, lp2_,
										rfx_, rfy_,
										rcx_, rcy_,
										rk1_, rk2_, rk3_,
										rp1_, rp2_);
	problem.AddResidualBlock(cost_function[j], NULL,target_->pose_.pb_pose);
    } // end target size matches observation number
    }// end if get observations successful
  }// end for each camera location
  
  // set up and solve the problem
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);
  if(summary.termination_type != ceres::NO_CONVERGENCE){
    double initial_cost = summary.initial_cost/(target_->num_points_*2);
    double final_cost = summary.final_cost/(target_->num_points_*2);
    ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
    target_->pose_.show("Target Pose");
    if(final_cost <= req.allowable_cost_per_observation){
      res.target_pose.position.x = target_->pose_.x;
      res.target_pose.position.y = target_->pose_.y;
      res.target_pose.position.z = target_->pose_.z;
      res.final_cost_per_observation  = final_cost;
      target_->pose_.getQuaternion(res.target_pose.orientation.x,
				   res.target_pose.orientation.y, 
				   res.target_pose.orientation.z,
				   res.target_pose.orientation.w);
      return true;
    }
    else{
      res.final_cost_per_observation  = final_cost;
      ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
      return(false);
    }
  }
}

void StereoLocatorService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  shared_ptr<industrial_extrinsic_cal::Target>(new industrial_extrinsic_cal::Target());
  target_->is_moving_ = true;
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ =  2;
  target_->circle_grid_parameters_.pattern_rows    = rows;
  target_->circle_grid_parameters_.pattern_cols    = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.is_symmetric    = true;
  target_->circle_grid_parameters_.spacing         = spacing; 
  target_->generatePoints();  // create a grid of points using the provided rows, cols, spacing and diameter
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_locate_service");
  ros::NodeHandle node_handle;
  StereoLocatorService stereo_loc(node_handle);


  ros::spin();
  ros::waitForShutdown();
  return 0;
}
