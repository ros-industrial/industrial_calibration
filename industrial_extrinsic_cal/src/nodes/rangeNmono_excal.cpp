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

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h> 
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp> 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_extrinsic_cal/calibrationAction.h>
#include <industrial_extrinsic_cal/calibrate.h>
#include <industrial_extrinsic_cal/covariance.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Point3d;
class RangeNmonoExCalService 
{
public:

  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::calibrationAction> CalibrationActionServer;

  RangeNmonoExCalService(ros::NodeHandle nh);
  ~RangeNmonoExCalService()  {  } ;
  bool executeCallBack( industrial_extrinsic_cal::calibrate::Request &req, industrial_extrinsic_cal::calibrate::Response &res);
  bool actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal);
  void  initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void initMonoCamera();
  void initRangeCamera();
private:
  ros::NodeHandle nh_;
  ros::ServiceServer range_excal_server_;
  shared_ptr<Target> target_;
  shared_ptr<industrial_extrinsic_cal::Camera> range_camera_;
  shared_ptr<industrial_extrinsic_cal::Camera> mono_camera_;
  string range_camera_name_;
  string range_image_topic_;
  string range_camera_frame_;
  string range_camera_mounting_frame_;
  string range_cloud_topic_;
  bool has_rroi_;
  Roi range_roi_;

  string mono_camera_name_;
  string mono_image_topic_;
  string mono_camera_frame_;
  string mono_camera_mounting_frame_;
  bool has_mroi_;
  Roi mono_roi_;

  int target_rows_;
  int target_cols_;


  CalibrationActionServer action_server_;
};

RangeNmonoExCalService::RangeNmonoExCalService(ros::NodeHandle nh):
action_server_(nh_,"run_calibration",boost::bind(&RangeNmonoExCalService::actionCallback, this, _1), false)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  if(!pnh.getParam( "range_image_topic", range_image_topic_)){
    ROS_ERROR("Must set param:  range_image_topic");
  }
  if(!pnh.getParam( "range_cloud_topic", range_cloud_topic_)){
    ROS_ERROR("Must set param: range_cloud_topic");
  }
  if(!pnh.getParam( "range_camera_name", range_camera_name_)){
    ROS_ERROR("Must set param: range_camera_name");
  }
  if(!pnh.getParam( "range_camera_frame", range_camera_frame_)){
    ROS_ERROR("Must set param: range_camera_frame");
  }
  if(!pnh.getParam( "range_camera_mounting_frame", range_camera_mounting_frame_)){
    ROS_ERROR("Must set param: range_camera_mounting_frame");
  }
  if(pnh.getParam( "range_ROI_xmin", range_roi_.x_min) && 
     pnh.getParam( "range_ROI_ymin", range_roi_.y_min) &&
     pnh.getParam( "range_ROI_xmax", range_roi_.x_max) &&
     pnh.getParam( "range_ROI_ymax", range_roi_.y_max) ) {
    has_rroi_=true;
  }
  else{
    has_rroi_ = false;
    ROS_INFO("no range camera ROI provided, using entire image");
  }

  if(!pnh.getParam( "mono_image_topic", mono_image_topic_)){
    ROS_ERROR("Must set param:  mono_image_topic");
  }
  if(!pnh.getParam( "mono_camera_name", mono_camera_name_)){
    ROS_ERROR("Must set param: mono_camera_name");
  }
  if(!pnh.getParam( "mono_camera_frame", mono_camera_frame_)){
    ROS_ERROR("Must set param: mono_camera_frame");
  }
  if(!pnh.getParam( "mono_camera_mounting_frame", mono_camera_mounting_frame_)){
    ROS_ERROR("Must set param: mono_camera_mounting_frame");
  }
  if(pnh.getParam( "mono_ROI_xmin", mono_roi_.x_min) && 
     pnh.getParam( "mono_ROI_ymin", mono_roi_.y_min) &&
     pnh.getParam( "mono_ROI_xmax", mono_roi_.x_max) &&
     pnh.getParam( "mono_ROI_ymax", mono_roi_.y_max) ) {
    has_mroi_=true;
  }
  else{
    has_mroi_ = false;
    ROS_INFO("no mono camera ROI provided, using entire image");
  }
  

  int rows, cols;
  double diameter, spacing;
  if(!pnh.getParam( "target_rows", target_rows_)){
    ROS_ERROR("Must set param:  target_rows");
  }
  if(!pnh.getParam( "target_cols", target_cols_)){
    ROS_ERROR("Must set param:  target_cols");
  }
  if(!pnh.getParam( "target_circle_dia", diameter)){
    ROS_ERROR("Must set param:  target_circle_dia");
  }
  if(!pnh.getParam( "target_spacing", spacing)){
    ROS_ERROR("Must set param:  target_spacing");
  }
  initMCircleTarget(target_rows_, target_cols_, diameter, spacing);
  
  std::string service_name;
  if(!pnh.getParam("service_name", service_name)){
    service_name = "RangeNmonoExCalService";
  }
  industrial_extrinsic_cal::CameraParameters range_cp;
  industrial_extrinsic_cal::CameraParameters mono_cp;
  range_camera_ = shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera(range_camera_name_, range_cp, false));
  mono_camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera(mono_camera_name_, mono_cp, false));
  initMonoCamera();
  initRangeCamera();

  // use the same service type as ususal for calibration, no need to create a new one
  range_excal_server_ =nh.advertiseService(service_name.c_str(), &RangeNmonoExCalService::executeCallBack, this);
  action_server_.start();
}

bool RangeNmonoExCalService::actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal)
{
  industrial_extrinsic_cal::calibrate::Request request;
  industrial_extrinsic_cal::calibrate::Response response;
  request.allowable_cost_per_observation = goal->allowable_cost_per_observation;
  if(executeCallBack(request, response)){
    action_server_.setSucceeded();
    return(true);
  }
  action_server_.setAborted();
  return(false);
}

bool RangeNmonoExCalService::executeCallBack( industrial_extrinsic_cal::calibrate::Request &req, 
					 industrial_extrinsic_cal::calibrate::Response &res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;
  industrial_extrinsic_cal::Cost_function cost_type; // don't need to set because this node assumes its cost type

  range_camera_->pullTransform();
  range_camera_->camera_observer_->clearObservations();
  range_camera_->camera_observer_->clearTargets();
  range_camera_->camera_observer_->addTarget(target_, range_roi_, cost_type);
  range_camera_->camera_observer_->triggerCamera();


  while (!range_camera_->camera_observer_->observationsDone()) ;
  range_camera_->camera_observer_->getObservations(camera_observations);

  int num_range_observations = (int) camera_observations.size();
  if(num_range_observations != target_rows_* target_cols_){
    ROS_ERROR("Range Camera Extrinisc Calibration: target not found. Only %d points", num_range_observations);
  }

  // get the range data from the 3D camera
  boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
  msg  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(range_cloud_topic_, ros::Duration(10));
  ROS_INFO("Received point cloud of size %d X %d", msg->width, msg->height);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::fromROSMsg(*msg, *cloud);

  ROS_INFO("Setting up the problem");
  Problem problem;
  for(int i=0; i<num_range_observations; i++){
    // the x,y location of the center of the region is found with sub-pixel accuracy
    // therefore the position in x,y,z of this point in space must be interpolated between the four surrounding point cloud points
    double image_x = camera_observations[i].image_loc_x; 
    double image_y = camera_observations[i].image_loc_y;
    unsigned int fx = floor(image_x); 
    unsigned int fy = floor(image_y);
    unsigned int cx = ceil(image_x);
    unsigned int cy = ceil(image_y);
    int height = cloud->height; // here we are assuming the height and width of the image and the point cloud are identical
    int width = cloud->width;// and that there is a one to one correspondence between pixels and cloud points
    Point3d tpoint; // target point
    tpoint.x = target_->pts_[camera_observations[i].point_id].x;
    tpoint.y = target_->pts_[camera_observations[i].point_id].y;
    tpoint.z = target_->pts_[camera_observations[i].point_id].z;
    // four quadrants for interpolation
    pcl::PointXYZ pff(cloud->points[fx + fy*width]);
    pcl::PointXYZ pfc(cloud->points[fx + cy*width]);
    pcl::PointXYZ pcf(cloud->points[cx + fy*width]);
    pcl::PointXYZ pcc(cloud->points[cx + cy*width]);
    double pctx = image_x - fx;// percentages since floor is always less than image_x by less than 1
    double pcty = image_y - fy;
    pcl::PointXYZ pt1(pff.x + pctx*(pfc.x-pff.x), pff.y + pctx*(pfc.y-pff.y), pff.z+ pctx*(pfc.z-pff.z)); // interpolate along x with y low
    pcl::PointXYZ pt2(pfc.x + pctx*(pcf.x-pcc.x), pfc.y + pctx*(pcf.y-pcc.y), pfc.z+ pctx*(pcf.z-pcc.z)); // interpolate along x with y low
    pcl::PointXYZ pt3(pt1.x + pcty*(pt2.x-pt1.x), pt1.y + pcty*(pt2.y-pt1.y), pt1.z+ pcty*(pt2.z-pt1.z)); // interp along y between p1&p2
    
    ROS_INFO("image(%f %f) pt3(%f %f %f), tpt(%f %f %f)", image_x, image_y, pt3.x, pt3.y, pt3.z, tpoint.x, tpoint.y, tpoint.z);
    // using the image_location x and y, determine the best estimate of x,y,z
    CostFunction* cost_function =
      industrial_extrinsic_cal::RangeSensorExtrinsicCal::Create(pt3.x, pt3.y, pt3.z, tpoint);
    problem.AddResidualBlock(cost_function, NULL , range_camera_->camera_parameters_.pb_extrinsics);
  }

  mono_camera_->pullTransform();
  mono_camera_->camera_observer_->clearObservations();
  mono_camera_->camera_observer_->clearTargets();
  mono_camera_->camera_observer_->addTarget(target_, mono_roi_, cost_type);
  mono_camera_->camera_observer_->triggerCamera();
  while (!mono_camera_->camera_observer_->observationsDone()) ;
  camera_observations.clear();
  mono_camera_->camera_observer_->getObservations(camera_observations);

  int num_mono_observations = (int) camera_observations.size();
  if(num_mono_observations != target_rows_* target_cols_){
    ROS_ERROR("Mono Camera Extrinisc Calibration: target not found. Only %d points", num_mono_observations);
  }

  double fx,fy,cx,cy;
  fx = mono_camera_->camera_parameters_.focal_length_x;
  fy = mono_camera_->camera_parameters_.focal_length_y;
  cx = mono_camera_->camera_parameters_.center_x;
  cy = mono_camera_->camera_parameters_.center_y;
  for(int i=0; i<num_mono_observations; i++){
    double image_x = camera_observations[i].image_loc_x; 
    double image_y = camera_observations[i].image_loc_y;
    Point3d tpoint; // target point
    tpoint.x = target_->pts_[camera_observations[i].point_id].x;
    tpoint.y = target_->pts_[camera_observations[i].point_id].y;
    tpoint.z = target_->pts_[camera_observations[i].point_id].z;
    CostFunction* cost_function =
      industrial_extrinsic_cal::CameraReprjErrorPK::Create(image_x, image_y, fx, fy, cx, cy, tpoint);
    problem.AddResidualBlock(cost_function, NULL , mono_camera_->camera_parameters_.pb_extrinsics);
  }
  if(num_range_observations+num_mono_observations == 0){
    ROS_ERROR("No Observations, can't calibrate either sensor");
    return(false);
  }

  ROS_INFO("Solving the problem");
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 25;
  ceres::Solve(options, &problem, &summary);
  ROS_INFO("Ceres Solve Finished!!" );

  if(summary.termination_type != ceres::NO_CONVERGENCE ){
    double error_per_observation = summary.final_cost/(num_mono_observations+num_range_observations);
    res.cost_per_observation  = error_per_observation;
    ROS_INFO("cost per observation = %f", error_per_observation);
    if(error_per_observation <= req.allowable_cost_per_observation){
      range_camera_->pushTransform();
      mono_camera_->pushTransform();
      return true;
    }
    else{
      ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
      return(false);
    }
  }
  ROS_ERROR("NO CONVERGENCE");
  return(false);
}

void RangeNmonoExCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  make_shared<industrial_extrinsic_cal::Target>();
  target_->is_moving_ = true;
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ =  pattern_options::ModifiedCircleGrid;
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
void  RangeNmonoExCalService::initMonoCamera()
{
  // set mono_camera's initial conditions by pulling from ti
  shared_ptr<industrial_extrinsic_cal::TransformInterface>  mono_ti = 
    shared_ptr<industrial_extrinsic_cal::ROSSimpleCameraCalTInterface>
    ( new industrial_extrinsic_cal::ROSSimpleCameraCalTInterface(mono_camera_frame_,  mono_camera_mounting_frame_));
  std::string ref_frame("dummy");
  mono_ti->setReferenceFrame(ref_frame);
  mono_camera_->setTransformInterface(mono_ti);

  // create the observers and get the camera info data if available, we only use it to set ROI when not set by rosparam
  mono_camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(mono_image_topic_, mono_camera_name_));
  double fx,fy,cx,cy,k1,k2,k3,p1,p2;
  int width,height;
  mono_camera_->camera_observer_->pullCameraInfo(fx,fy,cx,cy,k1,k2,k3,p1,p2,width,height);
  mono_camera_->camera_parameters_.focal_length_x = fx;
  mono_camera_->camera_parameters_.focal_length_y = fy;
  mono_camera_->camera_parameters_.center_x          = cx;
  mono_camera_->camera_parameters_.center_y          = cy;
  mono_camera_->camera_parameters_.distortion_k1   = k1;
  mono_camera_->camera_parameters_.distortion_k2   = k2;
  mono_camera_->camera_parameters_.distortion_k3   = k3;
  mono_camera_->camera_parameters_.distortion_p1   = p1;
  mono_camera_->camera_parameters_.distortion_p2   = p2;
  mono_camera_->camera_parameters_.width              = width;
  mono_camera_->camera_parameters_.height             = height;
  // set roi if not set by ros parameters
  if(!has_mroi_){
    mono_roi_.x_min = 0;
    mono_roi_.y_min = 0;
    mono_roi_.x_max = mono_camera_->camera_parameters_.width;
    mono_roi_.y_max = mono_camera_->camera_parameters_.height;
  }
  mono_camera_->camera_observer_->clearObservations();
  mono_camera_->camera_observer_->clearTargets();

}

void  RangeNmonoExCalService::initRangeCamera()
{
 // set range_camera's initial conditions by pulling from ti
  shared_ptr<industrial_extrinsic_cal::TransformInterface>  range_ti = 
    shared_ptr<industrial_extrinsic_cal::ROSSimpleCameraCalTInterface>
    (new industrial_extrinsic_cal::ROSSimpleCameraCalTInterface(range_camera_frame_,  range_camera_mounting_frame_));
  std::string ref_frame("dummy");
  range_ti->setReferenceFrame(ref_frame);
  range_camera_->setTransformInterface(range_ti);

  // create the observers and get the camera info data if available, we only use it to set ROI when not set by rosparam
  range_camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(range_image_topic_, range_camera_name_));
  double fx,fy,cx,cy,k1,k2,k3,p1,p2;
  int width,height;
  range_camera_->camera_observer_->pullCameraInfo(fx,fy,cx,cy,k1,k2,k3,p1,p2,width,height);
  range_camera_->camera_parameters_.focal_length_x = fx;
  range_camera_->camera_parameters_.focal_length_y = fy;
  range_camera_->camera_parameters_.center_x          = cx;
  range_camera_->camera_parameters_.center_y          = cy;
  range_camera_->camera_parameters_.distortion_k1   = k1;
  range_camera_->camera_parameters_.distortion_k2   = k2;
  range_camera_->camera_parameters_.distortion_k3   = k3;
  range_camera_->camera_parameters_.distortion_p1   = p1;
  range_camera_->camera_parameters_.distortion_p2   = p2;
  range_camera_->camera_parameters_.width              = width;
  range_camera_->camera_parameters_.height             = height;
  // set roi if not set by ros parameters
  if(!has_rroi_){
    range_roi_.x_min = 0;
    range_roi_.y_min = 0;
    range_roi_.x_max = range_camera_->camera_parameters_.width;
    range_roi_.y_max = range_camera_->camera_parameters_.height;
  }
 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_excal_service");
  ros::NodeHandle node_handle;
  RangeNmonoExCalService range_excal(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
