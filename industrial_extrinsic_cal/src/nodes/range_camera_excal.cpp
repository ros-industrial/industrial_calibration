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
class RangeExCalService 
{
public:

  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::calibrationAction> CalibrationActionServer;

  RangeExCalService(ros::NodeHandle nh);
  ~RangeExCalService()  {  } ;
  bool executeCallBack( industrial_extrinsic_cal::calibrate::Request &req, industrial_extrinsic_cal::calibrate::Response &res);
  bool actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal);
  void  initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void interpolate(double l, double m, double u, double lv, double &mv, double uv)
  {
    mv = lv +  (m-l)/(u-l)*(uv-lv);
  }
private:
  ros::NodeHandle nh_;
  ros::ServiceServer range_excal_server_;
  shared_ptr<Target> target_;
  shared_ptr<industrial_extrinsic_cal::Camera> camera_;
  string camera_name_;
  string image_topic_;
  string camera_frame_;
  string camera_mounting_frame_;
  string cloud_topic_;
  int image_height_;
  int image_width_;
  int target_type_;
  int target_rows_;
  int target_cols_;
  Roi roi_;
  CalibrationActionServer action_server_;
};

RangeExCalService::RangeExCalService(ros::NodeHandle nh):
action_server_(nh_,"run_calibration",boost::bind(&RangeExCalService::actionCallback, this, _1), false)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  int rows, cols;
  double diameter, spacing;
  if(!pnh.getParam( "image_topic", image_topic_)){
    ROS_ERROR("Must set param:  image_topic");
  }
  if(!pnh.getParam( "cloud_topic", cloud_topic_)){
    ROS_ERROR("Must set param:  cloud_topic");
  }
  if(!pnh.getParam( "camera_name", camera_name_)){
    ROS_ERROR("Must set param: camera_name");
  }
  if(!pnh.getParam( "image_height", image_height_)){
    ROS_ERROR("Must set param: image_height");
  }
  if(!pnh.getParam( "image_width", image_width_)){
    ROS_ERROR("Must set param: image_width");
  }
  if(!pnh.getParam( "camera_frame", camera_frame_)){
    ROS_ERROR("Must set param: camera");
  }
  if(!pnh.getParam( "camera_mounting_frame", camera_mounting_frame_)){
    ROS_ERROR("Must set param: camera_mounting_frame");
  }
  if(!pnh.getParam( "ROI_xmin", roi_.x_min)){
    ROS_ERROR("Must set param: ROI_xmin");
  }
  if(!pnh.getParam( "ROI_ymin", roi_.y_min)){
    ROS_ERROR("Must set param: ROI_ymin");
  }
  if(!pnh.getParam( "ROI_xmax", roi_.x_max)){
    ROS_ERROR("Must set param: ROI_xmax");
  }
  if(!pnh.getParam( "ROI_ymax", roi_.y_max)){
    ROS_ERROR("Must set param: ROI_ymax");
  }

  target_type_ == pattern_options::ModifiedCircleGrid;
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
    service_name = "RangeExCalService";
  }
  industrial_extrinsic_cal::CameraParameters temp_parameters;
  temp_parameters.height = image_height_;
  temp_parameters.width = image_width_;
  camera_ = shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera(camera_name_, temp_parameters, false));
  // use the same service type as ususal for calibration, no need to create a new one
  range_excal_server_ =nh.advertiseService(service_name.c_str(), &RangeExCalService::executeCallBack, this);
  action_server_.start();
}

bool RangeExCalService::actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal)
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
bool RangeExCalService::executeCallBack( industrial_extrinsic_cal::calibrate::Request &req, 
					 industrial_extrinsic_cal::calibrate::Response &res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;

  // set initial conditions to something that should converge when looking more or less straight at a target
  shared_ptr<industrial_extrinsic_cal::TransformInterface>  temp_ti = 
    shared_ptr<industrial_extrinsic_cal::ROSSimpleCameraCalTInterface>(new industrial_extrinsic_cal::ROSSimpleCameraCalTInterface(camera_frame_,  camera_mounting_frame_));
  std::string ref_frame("dummy");
  temp_ti->setReferenceFrame(ref_frame);
  camera_->setTransformInterface(temp_ti);
  camera_->pullTransform();

  camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(image_topic_, camera_name_));
  camera_->camera_observer_->clearObservations();
  camera_->camera_observer_->clearTargets();

  industrial_extrinsic_cal::Cost_function cost_type; // don't need to set because this node assumes its cost type
  
  // get the observations from the intensity image
  camera_->camera_observer_->addTarget(target_, roi_, cost_type);
  camera_->camera_observer_->triggerCamera();
  while (!camera_->camera_observer_->observationsDone()) ;
  camera_->camera_observer_->getObservations(camera_observations);

  int num_observations = (int) camera_observations.size();
  if(num_observations != target_rows_* target_cols_){
    ROS_ERROR("Range Camera Extrinisc Calibration: target not found. Only %d points", num_observations);
    return(false);
  }

  // get the range data from the 3D camera
  boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
  msg  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic_, ros::Duration(10));
  ROS_INFO("Received point cloud of size %d X %d", msg->width, msg->height);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::fromROSMsg(*msg, *cloud);

  ROS_INFO("Setting up the problem");
  Problem problem;
  for(int i=0; i<num_observations; i++){
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
    tpoint.x =target_->pts_[i].x;
    tpoint.y = target_->pts_[i].y;
    tpoint.z = target_->pts_[i].z; 
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
    
    ROS_INFO("image(%f %f) pff(%f %f %f), tpt(%f %f %f)", image_x, image_y, pff.x, pff.y, pff.z, tpoint.x, tpoint.y, tpoint.z);
    // using the image_location x and y, determine the best estimate of x,y,z
    CostFunction* cost_function =
      industrial_extrinsic_cal::RangeSensorExtrinsicCal::Create(pt3.x, pt3.y, pt3.z, tpoint);
    problem.AddResidualBlock(cost_function, NULL , camera_->camera_parameters_.pb_extrinsics);
  }

  ROS_INFO("Solving the problem");
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 15;
  ceres::Solve(options, &problem, &summary);
  ROS_INFO("Ceres Solve Finished!!" );

  if(summary.termination_type != ceres::NO_CONVERGENCE
     ){
    double error_per_observation = summary.final_cost/num_observations;
    res.cost_per_observation  = error_per_observation;
    ROS_INFO("cost per observation = %f", error_per_observation);
    if(error_per_observation <= req.allowable_cost_per_observation){
      camera_->pushTransform();
      return true;
    }
    else{
      ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
      return(false);
    }
  }
}

void RangeExCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  make_shared<industrial_extrinsic_cal::Target>();
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
  ros::init(argc, argv, "range_excal_service");
  ros::NodeHandle node_handle;
  RangeExCalService range_excal(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
