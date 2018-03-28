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
#include <intrinsic_cal/rail_scal_run.h>
#include <robo_cylinder/HomeCmd.h>
#include <robo_cylinder/MoveMeters.h>
#include <robo_cylinder/MovePulses.h>
#include <robo_cylinder/PowerIO.h>
#include <robo_cylinder/StatusUpdate.h>
#include <robo_cylinder/VelAcc.h>
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

class RobocylSCalService 
{
public:
  RobocylSCalService(ros::NodeHandle nh);
  ~RobocylSCalService()  {  } ;
  bool executeCallBack( intrinsic_cal::rail_scal_run::Request &req, intrinsic_cal::rail_scal_run::Response &res);
  bool MoveAndReportPose(double rail_position, Pose6d &LP, Pose6d &RP);
  void  initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void leftCameraCallback(const sensor_msgs::Image& image);
  void rightCameraCallback(const sensor_msgs::Image& image);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer rail_cal_server_;
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;
  ros::Publisher left_center_pub_;
  ros::Publisher right_center_pub_;
  ros::ServiceClient move_client_; /**< a client for calling the service to move the robo-cylinder to a new location */
  ros::ServiceClient power_client_; /**< a client for calling the service to turn on the robo-cylinder */
  ros::ServiceClient home_client_; /**< a client for calling the service to move robo-cylinder to its home position */
  shared_ptr<Target> target_;
  shared_ptr<Camera> left_camera_;
  shared_ptr<Camera> right_camera_;
  double focal_length_x_;
  double focal_length_y_;
  double center_x_;
  double center_y_;
  string left_image_topic_;
  string left_camera_name_;
  string right_image_topic_;
  string right_camera_name_;
  int target_type_;
  int target_rows_;
  int target_cols_;
  double circle_spacing_;
  double circle_diameter_;
  int num_camera_locations_;
  double camera_spacing_;
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

};

RobocylSCalService::RobocylSCalService(ros::NodeHandle nh)
{
  
  nh_ = nh;
  ros::NodeHandle pnh("~");
  allowable_cost_per_observation_ = 1.0;

  if(!pnh.getParam( "left_image_topic", left_image_topic_)){
    ROS_ERROR("Must set param:  left_image_topic");
  }

  if(!pnh.getParam( "left_camera_name", left_camera_name_)){
    ROS_ERROR("Must set param:  left_camera_name");
  }
  if(!pnh.getParam( "right_image_topic", right_image_topic_)){
    ROS_ERROR("Must set param:  right_image_topic");
  }

  if(!pnh.getParam( "right_camera_name", right_camera_name_)){
    ROS_ERROR("Must set param:  right_camera_name");
  }

  target_type_ == pattern_options::ModifiedCircleGrid;

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
  if(!pnh.getParam( "num_camera_locations", num_camera_locations_)){
    ROS_ERROR("Must set param:  num_camera_locations");
  }
  if(!pnh.getParam( "camera_spacing", camera_spacing_)){
    ROS_ERROR("Must set param:  camera_spacing");
  }
  if(!pnh.getParam( "target_to_rail_distance", D0_)){
    ROS_ERROR("Must set param:  target_to_rail_distance");
  }
  
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
  string move_client_name("/move_meters");
  if(!pnh.getParam( "move_client", move_client_name)){
    ROS_WARN("move_client = %s", move_client_name.c_str());
  }
  string power_client_name("/power_io");
  if(!pnh.getParam( "power_client", power_client_name)){
    ROS_WARN("power_client = %s", power_client_name.c_str());
  }
  string home_client_name("/home");
  if(!pnh.getParam( "home_client", home_client_name)){
    ROS_WARN("home_client = %s", home_client_name.c_str());
  }

  move_client_  = nh.serviceClient<robo_cylinder::MoveMeters>(move_client_name);
  power_client_  = nh.serviceClient<robo_cylinder::PowerIO>(power_client_name);
  home_client_  = nh.serviceClient<robo_cylinder::HomeCmd>(home_client_name);

  u_int32_t queue_size = 5;
  left_sub_  = nh_.subscribe(left_image_topic_, queue_size, &RobocylSCalService::leftCameraCallback, this);
  right_sub_ = nh_.subscribe(right_image_topic_, queue_size, &RobocylSCalService::rightCameraCallback, this);
  left_center_pub_  = nh_.advertise<sensor_msgs::Image>("left_image_center", 1);
  right_center_pub_ = nh_.advertise<sensor_msgs::Image>("right_image_center", 1);

  if(!use_quaternion)
  {
    tf::Matrix3x3 m;
    m[0][0] = 1; m[0][1] =   0; m[0][2] = 0;
    m[1][0] = 0; m[1][1] = -1.; m[1][2] = 0;
    m[2][0] = 0; m[2][1] =   0; m[2][2] = -1;
    Pose6d Ptemp;
    Ptemp.setBasis(m);
    Ptemp.setOrigin(-0.1, -0.1, D0_);
    Ptemp.getQuaternion(qx_, qy_, qz_, qw_);
    Ptemp.show("initial pose");
    ROS_WARN("parameters qx, qy, qz, and qw not provided, using default values of (%.2f, %.2f, %.2f, %.2f)", qx_, qy_, qz_, qw_);
  }

  bool is_moving = true;
  left_camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera("left_camera", left_camera_parameters_, is_moving));
  right_camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera("right_camera", right_camera_parameters_, is_moving));
  left_camera_->trigger_ = shared_ptr<NoWaitTrigger>(new NoWaitTrigger());
  right_camera_->trigger_ = shared_ptr<NoWaitTrigger>(new NoWaitTrigger());
  left_camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(left_image_topic_, left_camera_name_));
  right_camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(right_image_topic_, right_camera_name_));
  sleep(10); // wait for camera to come up or else this will fail
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
  lk1_ = left_camera_->camera_parameters_.distortion_k1;
  lk2_ = left_camera_->camera_parameters_.distortion_k2;
  lk3_ = left_camera_->camera_parameters_.distortion_k3;
  lp1_ = left_camera_->camera_parameters_.distortion_p1;
  lp2_ = left_camera_->camera_parameters_.distortion_p2;
  rfx_ = right_camera_->camera_parameters_.focal_length_x;
  rfy_ = right_camera_->camera_parameters_.focal_length_y;
  rcx_ = right_camera_->camera_parameters_.center_x;
  rcy_ = right_camera_->camera_parameters_.center_y;
  rk1_ = right_camera_->camera_parameters_.distortion_k1;
  rk2_ = right_camera_->camera_parameters_.distortion_k2;
  rk3_ = right_camera_->camera_parameters_.distortion_k3;
  rp1_ = right_camera_->camera_parameters_.distortion_p1;
  rp2_ = right_camera_->camera_parameters_.distortion_p2;
  left_roi_.x_min = 0;
  left_roi_.y_min = 0;
  left_roi_.x_max = left_image_width_;
  left_roi_.y_max = left_image_height_;
  right_roi_.x_min = 0;
  right_roi_.y_min = 0;
  right_roi_.x_max = right_image_width_;
  right_roi_.y_max = right_image_height_;


  ROS_INFO("left camera info focal:%f %f center:%f %f  radial:%f %f %f tang: %f %f",
            left_camera_->camera_parameters_.focal_length_x,
            left_camera_->camera_parameters_.focal_length_y,
            left_camera_->camera_parameters_.center_x,
            left_camera_->camera_parameters_.center_y,
            left_camera_->camera_parameters_.distortion_k1,
            left_camera_->camera_parameters_.distortion_k2,
            left_camera_->camera_parameters_.distortion_k3,
            left_camera_->camera_parameters_.distortion_p1,
            left_camera_->camera_parameters_.distortion_p2);

  ROS_INFO("right camera info focal:%f %f center:%f %f  radial:%f %f %f tang: %f %f",
            right_camera_->camera_parameters_.focal_length_x,
            right_camera_->camera_parameters_.focal_length_y,
            right_camera_->camera_parameters_.center_x,
            right_camera_->camera_parameters_.center_y,
            right_camera_->camera_parameters_.distortion_k1,
            right_camera_->camera_parameters_.distortion_k2,
            right_camera_->camera_parameters_.distortion_k3,
            right_camera_->camera_parameters_.distortion_p1,
            right_camera_->camera_parameters_.distortion_p2);

  initMCircleTarget(target_rows_, target_cols_, circle_diameter_, circle_spacing_);
 
  rail_cal_server_ = nh_.advertiseService( "RobocylSCalService", &RobocylSCalService::executeCallBack, this);
  
}
// simply publishes the left camera image with a circle drawn at the center
void RobocylSCalService::leftCameraCallback(const sensor_msgs::Image &image)
{
  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(image, image.encoding);

  cv::Mat mod_img = bridge->image;
  cv::circle(mod_img, cv::Point2d(image.width / 2.0, image.height / 2.0), 4, cv::Scalar(255,0,0), 2);
  bridge->image = mod_img;
  left_image_height_ = image.height;
  left_image_width_ = image.width;
  sensor_msgs::Image out_img;
  bridge->toImageMsg(out_img);
  left_center_pub_.publish(out_img);

}
// simply publishes the right camera image with a circle drawn at the center
void RobocylSCalService::rightCameraCallback(const sensor_msgs::Image &image)
{
  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(image, image.encoding);

  cv::Mat mod_img = bridge->image;
  cv::circle(mod_img, cv::Point2d(image.width / 2.0, image.height / 2.0), 4, cv::Scalar(255,0,0), 2);
  bridge->image = mod_img;
  right_image_height_ = image.height;
  right_image_width_ = image.width;
  sensor_msgs::Image out_img;
  bridge->toImageMsg(out_img);
  right_center_pub_.publish(out_img);

}

// moves the stage through a series of positions and solves for the tranform between the two cameras
bool RobocylSCalService::executeCallBack( intrinsic_cal::rail_scal_run::Request &req, intrinsic_cal::rail_scal_run::Response &res)
{
  allowable_cost_per_observation_ = req.allowable_cost_per_observation;
  ros::NodeHandle nh;
  CameraObservations left_camera_observations;
  CameraObservations right_camera_observations;
  robo_cylinder::MoveMeters::Request mm_request; /**< request when transform is part of a mutable set */
  robo_cylinder::MoveMeters::Response mm_response; /**< request when transform is part of a mutable set */
  robo_cylinder::PowerIO::Request pio_request; /**< request when transform is part of a mutable set */
  robo_cylinder::PowerIO::Response pio_response; /**< request when transform is part of a mutable set */
  robo_cylinder::HomeCmd::Request hc_request; /**< request when transform is part of a mutable set */
  robo_cylinder::HomeCmd::Response hc_response; /**< request when transform is part of a mutable set */

  int q=0; // counter for cost_function array that we create
  int num_observations   = target_rows_*target_cols_; // number of points on target
  int total_observations = 2*num_camera_locations_*num_observations; // each point observed twice, once by each camera in every position


  left_camera_->camera_observer_->clearObservations();
  left_camera_->camera_observer_->clearTargets();
  right_camera_->camera_observer_->clearObservations();
  right_camera_->camera_observer_->clearTargets();


  // turn power on to robo-cylinder, and move it home
  pio_request.io = 1;
  power_client_.call(pio_request, pio_response);
  home_client_.call(hc_request, hc_response);

  Pose6d TtoLC, TtoRC; // transforms points in target frame into either Camera1 or Camera2 frames
  Pose6d LCtoRC; // transforms point in left camera frame to right camera frame
  double travel_distance = (num_camera_locations_ -1)*camera_spacing_;

  // initialized desired LCtoRC using close poses
  if(!MoveAndReportPose(0.0,TtoLC, TtoRC)){
    if(!MoveAndReportPose(0.0,TtoLC, TtoRC)) return(false);
  }
  LCtoRC = TtoRC*(TtoLC.getInverse()); // this is the pose we want to refine in this optimization
  LCtoRC.show("initial LCtoRC stage = 0.0");
  
  // compute the motion vector of the rail in the first left camera coordinate frame
  Pose6d LC2toLC1; // transform from left camera position 1 to left camera position 2
  Pose6d TtoLC2, TtoRC2; // transform from target to left camera in position2
  if(!MoveAndReportPose(0.75,TtoLC2, TtoRC2)){
    if(!MoveAndReportPose(0.75,TtoLC2, TtoRC2)) return(false);
  }
  Pose6d LC2toRC2 = TtoRC2*(TtoLC2.getInverse());
  LC2toRC2.show("LC2toRC2 from stage = 0.75");

  Pose6d LC2toT  = TtoLC2.getInverse(); // this transform points in LC2 frame to target frame
  Pose6d LC2toLC = TtoLC*LC2toT; // this transforms points in LC2 frame into Target then into LC frame as one multiply
  tf::Vector3 mv = LC2toLC.getOrigin(); // the origin is the vector in C1 coordinates

  // check that the distance measured is similar to the requested move
  // sometimes the stage has problems, and thinks it's moved when it hasn't
  double dist = sqrt(mv.getX()*mv.getX() + mv.getY()*mv.getY() + mv.getZ()*mv.getZ());
  if(fabs(travel_distance -dist) > 0.9*travel_distance){
    ROS_ERROR("motion didn't occur correctly. Is initial focal length way off?");
  }
  mv.normalize();
  ROS_INFO("dist = %lf mv: %lf %lf %lf",dist, mv.getX(),mv.getY(),mv.getZ());

  // set up the problem
  industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::RailSCal;
  Problem problem;

  target_->pose_ = TtoLC; // initialize target's pose, don't know why this needs done, 
  std::vector<Pose6d> target_poses;
  tf::Matrix3x3 M = target_->pose_.getBasis();
  tf::Vector3   V = target_->pose_.getOrigin();
  for(int i=0;i<num_camera_locations_;i++){
    Pose6d temp_pose;
    target_poses.push_back(temp_pose);
    target_poses[i].setOrigin(V);
    target_poses[i].setBasis(M);
    target_poses[i].x +=i*.001;
  }

  CostFunction* cost_function[num_observations*num_camera_locations_];
  for(int i=0; i<num_camera_locations_; i++){
    double Dist = i*camera_spacing_;
    Point3d rail_position;
    rail_position.x = -Dist*mv.getX();
    rail_position.y = -Dist*mv.getY();
    rail_position.z = -Dist*mv.getZ();

    rail_position.x = 0;
    rail_position.y = 0;
    rail_position.z = 0;
    
    ROS_INFO("moving to %lf",Dist);
    mm_request.meters = Dist;
    move_client_.call(mm_request, mm_response); // this call blocks until camera is moved
    sleep(.5);
    // gather next image
    left_camera_->camera_observer_->clearTargets();
    left_camera_->camera_observer_->clearObservations();
    left_camera_->camera_observer_->addTarget(target_, left_roi_, cost_type);
    left_camera_->camera_observer_->triggerCamera();
    right_camera_->camera_observer_->clearTargets();
    right_camera_->camera_observer_->clearObservations();
    right_camera_->camera_observer_->addTarget(target_, right_roi_, cost_type);
    right_camera_->camera_observer_->triggerCamera();
    while (!left_camera_->camera_observer_->observationsDone() && !right_camera_->camera_observer_->observationsDone()) ;
    bool left_ok  = left_camera_->camera_observer_->getObservations(left_camera_observations);
    bool right_ok = right_camera_->camera_observer_->getObservations(right_camera_observations);
    if(left_ok && right_ok){
      ROS_INFO("Found %d observations in left image",(int) left_camera_observations.size());
      ROS_INFO("Found %d observations in right image",(int) right_camera_observations.size());
      int nleft = (int) left_camera_observations.size();
      int nright = (int) right_camera_observations.size();
      if(nleft != num_observations || nright != num_observations ){
	ROS_ERROR("Could not find target in image num_left_obs = %d num_rt_obs = %d", nleft, nright);
      }
      else{	// add a new cost to the problem for each observation
	for(int j=0; j<num_observations; j++){
	  double left_image_x = left_camera_observations[j].image_loc_x;
	  double left_image_y = left_camera_observations[j].image_loc_y;
	  Point3d left_point = target_->pts_[left_camera_observations[j].point_id]; // don't assume ordering from camera observer
	  double right_image_x = right_camera_observations[j].image_loc_x;
	  double right_image_y = right_camera_observations[j].image_loc_y;
	  Point3d right_point = target_->pts_[right_camera_observations[j].point_id]; // don't assume ordering from camera observer
	  cost_function[q] = industrial_extrinsic_cal::RailSCal::Create(left_image_x, left_image_y, left_point,
									right_image_x, right_image_y, right_point,
									rail_position,
									lfx_, lfy_,
									lcx_, lcy_,
									lk1_, lk2_, lk3_,
									lp1_, lp2_,
									rfx_, rfy_,
									rcx_, rcy_,
									rk1_, rk2_, rk3_,
									rp1_, rp2_);
	  problem.AddResidualBlock(cost_function[q], NULL,
				   LCtoRC.pb_pose,
				   //target_->pose_.pb_pose);
				   target_poses[i].pb_pose);
	  q++; // increment the cost function
	} // for each observation at this camera_location
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
    double initial_cost = summary.initial_cost/total_observations;
    double final_cost = summary.final_cost/total_observations;
    ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
    LCtoRC.show("LCtoRC");
    for(int i=0;i<num_camera_locations_;i++) {
      char msg[12];
      sprintf(msg,"target_pose%d",i);
      target_poses[i].show(msg);
    }
    if(final_cost <= req.allowable_cost_per_observation){
      res.right_camera_pose.position.x = LCtoRC.x;
      res.right_camera_pose.position.y = LCtoRC.y;
      res.right_camera_pose.position.z = LCtoRC.z;
      res.final_cost_per_observation  = final_cost;
      target_->pose_.getQuaternion(res.right_camera_pose.orientation.x,
				   res.right_camera_pose.orientation.y, 
				   res.right_camera_pose.orientation.z,
				   res.right_camera_pose.orientation.w);
      return true;
    }
    else{
      res.final_cost_per_observation  = final_cost;
      ROS_ERROR("allowable cost exceeded %f > %f", final_cost, req.allowable_cost_per_observation);
      return(false);
    }
  }
}

bool RobocylSCalService::MoveAndReportPose(double rail_position, Pose6d &LP, Pose6d &RP)
{
  ros::NodeHandle nh;
  CameraObservations left_camera_observations;
  CameraObservations right_camera_observations;
  robo_cylinder::MoveMeters::Request mm_request; /**< request when transform is part of a mutable set */
  robo_cylinder::MoveMeters::Response mm_response; /**< request when transform is part of a mutable set */
  int num_observations;
  int total_observations=0;

  // set the roi to the whole image
  industrial_extrinsic_cal::Cost_function cost_type;
  Problem problem;
  Pose6d left_pose;
  Pose6d right_pose;

  // set initial conditions,
  left_pose.setQuaternion(qx_, qy_, qz_, qw_);
  left_pose.setOrigin(0.011, 0.05, D0_-rail_position);
  right_pose.setQuaternion(qx_, qy_, qz_, qw_);
  right_pose.setOrigin(0.011, 0.05, D0_-rail_position);
  
  // Move camera into position
  ROS_INFO("moving to %lf",rail_position);
  mm_request.meters = rail_position;
  move_client_.call(mm_request, mm_response);
  sleep(1); // wait for vibrations to settle before collecting images

  // collect observations from both cameras
  left_camera_->camera_observer_->clearObservations();
  left_camera_->camera_observer_->clearTargets();
  left_camera_->camera_observer_->addTarget(target_, left_roi_, cost_type);
  left_camera_->camera_observer_->triggerCamera();
  while (!left_camera_->camera_observer_->observationsDone()) ;

  right_camera_->camera_observer_->clearObservations();
  right_camera_->camera_observer_->clearTargets();
  right_camera_->camera_observer_->addTarget(target_, right_roi_, cost_type);
  right_camera_->camera_observer_->triggerCamera();
  while (!right_camera_->camera_observer_->observationsDone()) ;

  // get the observations
  bool left_obs_ok  = left_camera_->camera_observer_->getObservations(left_camera_observations);
  bool right_obs_ok = right_camera_->camera_observer_->getObservations(right_camera_observations);

  // use the observations to create and solve a problem
  if(left_obs_ok && right_obs_ok){ 
    ROS_INFO("Found %d observations in left image",(int) left_camera_observations.size());
    ROS_INFO("Found %d observations in right image",(int) right_camera_observations.size());
    num_observations = (int) left_camera_observations.size();
    if(num_observations != target_rows_* target_cols_){
      ROS_ERROR("Target Locator could not find target %d in left image", num_observations);
      LP = left_pose;
      return(false);
    }

    num_observations = (int) right_camera_observations.size();
    if(num_observations != target_rows_* target_cols_){
      ROS_ERROR("Target Locator could not find target %d in right image", num_observations);
      RP = right_pose;
      return(false);
    }

    // add a new cost to the problem for each observation
    CostFunction* cost_function[2*num_observations];
    total_observations += num_observations;
    int q=0;
    for(int i=0; i<num_observations; i++){
      double left_image_x = left_camera_observations[i].image_loc_x;
      double left_image_y = left_camera_observations[i].image_loc_y;
      Point3d left_point = target_->pts_[left_camera_observations[i].point_id]; // don't assume ordering from camera observer
      cost_function[q] =  industrial_extrinsic_cal::DistortedCameraFinder::Create(left_image_x, left_image_y,
										    lfx_, lfy_, lcx_, lcy_, lk1_, lk2_, lk3_, lp1_, lp2_, left_point);
      problem.AddResidualBlock(cost_function[q++], NULL, left_pose.pb_pose);

      double right_image_x = right_camera_observations[i].image_loc_x;
      double right_image_y = right_camera_observations[i].image_loc_y;
      Point3d right_point = target_->pts_[right_camera_observations[i].point_id]; // don't assume ordering from camera observer
      cost_function[q] =  industrial_extrinsic_cal::DistortedCameraFinder::Create(right_image_x, right_image_y,
										    rfx_, rfy_, rcx_, rcy_, rk1_, rk2_, rk3_, rp1_, rp2_, right_point);
      problem.AddResidualBlock(cost_function[q++], NULL, right_pose.pb_pose);
    }
  }

  // set up and solve the problem
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);
  LP = left_pose;
  RP = right_pose;

  if(summary.termination_type != ceres::NO_CONVERGENCE){
    double initial_cost = summary.initial_cost/total_observations;
    double final_cost = summary.final_cost/total_observations;
    if(final_cost <= allowable_cost_per_observation_){
      ROS_INFO("Found Pose, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
      left_pose.show("Left Pose");
      right_pose.show("Right Pose");
      return(true);
    }
    else{
      ROS_ERROR("finding pose: allowable cost exceeded %f > %f", final_cost, allowable_cost_per_observation_);
      return(false);
    }
  }
  else{
    ROS_ERROR("finding pose: No Convergence");
    return(false);
  }
}

void RobocylSCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  shared_ptr<industrial_extrinsic_cal::Target>(new industrial_extrinsic_cal::Target());
  target_->is_moving_ = true;
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ =  2;
  target_->circle_grid_parameters_.pattern_rows =rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.is_symmetric = true; 
  // create a grid of points
  target_->pts_.clear();
  target_->num_points_ = rows*cols;
  for(int i=0; i<rows; i++){
    for(int j=0; j<cols; j++){
      Point3d point;
      point.x = j*spacing;
      point.y = (rows -1 -i)*spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rail_scal_service");
  ros::NodeHandle node_handle;
  RobocylSCalService rail_scal(node_handle);


  ros::spin();
  ros::waitForShutdown();
  return 0;
}
