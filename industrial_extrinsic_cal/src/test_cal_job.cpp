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
#include <industrial_extrinsic_cal/runtime_utils.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

using industrial_extrinsic_cal::CalibrationJob;
using std::string;

void print_AAtoH(double &x, double &y, double &z, double &tx, double &ty, double &tz);
void print_AAToHI(double x, double y, double z, double tx, double ty, double tz);
void generateMessages(std::vector<geometry_msgs::Pose> poses);
void pblockToPose(industrial_extrinsic_cal::P_BLOCK optimized_input);
ros::Publisher transform_pub1, transform_pub2, transform_pub3, transform_pub4;
std::vector<geometry_msgs::Pose> pose_msgs;
std::vector<tf::Transform> transforms;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_node");

  industrial_extrinsic_cal::ROSRuntimeUtils utils;
  ros::NodeHandle nh;
  transform_pub1= nh.advertise<geometry_msgs::PoseStamped>("camera1_pose", 1);
  transform_pub2= nh.advertise<geometry_msgs::PoseStamped>("camera2_pose", 1);
  transform_pub3= nh.advertise<geometry_msgs::PoseStamped>("target1_pose", 1);
  transform_pub4= nh.advertise<geometry_msgs::PoseStamped>("target2_pose", 1);

  utils.camera_file_="/home/cgomez/ros/hydro/catkin_ws/src/"
      "industrial_calibration/industrial_extrinsic_cal/yaml/test2_camera_def.yaml";
  utils.target_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test2_target_def.yaml";
  utils.caljob_file_="/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration"
      "/industrial_extrinsic_cal/yaml/test2_caljob_def.yaml";
  industrial_extrinsic_cal::CalibrationJob Cal_job(utils.camera_file_, utils.target_file_, utils.caljob_file_);
  //ROS_INFO_STREAM("hello world ");
  if (Cal_job.load())
  {
    ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
  }

  utils.world_frame_=Cal_job.getReferenceFrame();
  utils.camera_optical_frame_=Cal_job.getCameraOpticalFrame();
  utils.camera_intermediate_frame_=Cal_job.getCameraIntermediateFrame();
  utils.initial_extrinsics_ = Cal_job.getOriginalExtrinsics();
  utils.target_frame_=Cal_job.getTargetFrames();
  industrial_extrinsic_cal::P_BLOCK orig_extrinsics;
  tf::Transform tf_camera_orig;

  for (int k=0; k<utils.initial_extrinsics_.size(); k++ )
  {
    //ROS_INFO_STREAM("each camera extrinsics: "<<utils.initial_extrinsics_[k][0]<<" "<<utils.initial_extrinsics_[k][1]<<" "
      //            <<utils.initial_extrinsics_[k][2]<<" "<<utils.initial_extrinsics_[k][3]<<" "
        //          <<utils.initial_extrinsics_[k][4]<<" "<<utils.initial_extrinsics_[k][5]);
    orig_extrinsics=utils.initial_extrinsics_[k];
    ROS_INFO_STREAM("Original Camera "<<k);
    tf_camera_orig= utils.pblockToPose(orig_extrinsics);
    utils.initial_transforms_.push_back(tf_camera_orig);
  }
  utils.broadcasters_.resize(utils.initial_extrinsics_.size());
  ROS_INFO_STREAM("Size of broadcasters: "<<utils.broadcasters_.size());
  ROS_INFO_STREAM("Size of initial_transforms: "<<utils.initial_transforms_.size());
  ROS_INFO_STREAM("Size of cam_int_frame: "<<utils.camera_intermediate_frame_.size());
  ROS_INFO_STREAM("Size of target_frame: "<<utils.target_frame_.size());
  ROS_INFO_STREAM("Target frame: "<<utils.target_frame_[0]);
  for (int k=0; k<utils.broadcasters_.size(); k++ )
  {
    utils.broadcasters_[k].sendTransform(tf::StampedTransform(utils.initial_transforms_[k], ros::Time::now(),
                                                              utils.target_frame_[0], utils.camera_intermediate_frame_[k]));
  }

  if (Cal_job.run())
  {
    ROS_INFO_STREAM("Calibration job observations and optimization complete");
  }

  utils.calibrated_extrinsics_ = Cal_job.getExtrinsics();
  utils.target_poses_ = Cal_job.getTargetPose();
  ROS_DEBUG_STREAM("Size of optimized_extrinsics_: "<<utils.calibrated_extrinsics_.size());
  ROS_DEBUG_STREAM("Size of targets_: "<<utils.target_poses_.size());

  industrial_extrinsic_cal::P_BLOCK optimized_extrinsics, target;
  tf::Transform tf_camera, tf_target;
  for (int k=0; k<utils.calibrated_extrinsics_.size(); k++ )
  {
    optimized_extrinsics=utils.calibrated_extrinsics_[k];
    /*print_AAtoH(optimized_extrinsics[0], optimized_extrinsics[1], optimized_extrinsics[2],
                optimized_extrinsics[3], optimized_extrinsics[4], optimized_extrinsics[5]);
    print_AAToHI(optimized_extrinsics[0], optimized_extrinsics[1], optimized_extrinsics[2],
                 optimized_extrinsics[3], optimized_extrinsics[4], optimized_extrinsics[5]);*/
    ROS_INFO_STREAM("Optimized Camera "<<k);
    tf_camera=utils.pblockToPose(optimized_extrinsics);
    utils.calibrated_transforms_.push_back(tf_camera);
    //pblockToPose(optimized_extrinsics);
  }
  tf::Vector3 tf_cal=tf_camera.getOrigin();
  ROS_INFO_STREAM("Cal transform origin: "<<tf_cal.x()<<" "<<tf_cal.y()<<" "<<tf_cal.z());
  for (int k=0; k<utils.target_poses_.size(); k++ )
  {
    target=utils.target_poses_[k];
    ROS_INFO_STREAM("Optimized Target "<<k);
    tf_target=utils.pblockToPose(target);
    utils.target_transforms_.push_back(tf_target);
  }
  //generateMessages(pose_msgs);
  tf::StampedTransform temp_tf;
  for (int i=0; i<utils.calibrated_extrinsics_.size(); i++ )
  {
    try
    {
      utils.listener_.lookupTransform( utils.camera_optical_frame_[i],utils.camera_intermediate_frame_[i],
                                       ros::Time(0), temp_tf);
      utils.camera_internal_transforms_.push_back(temp_tf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
  ROS_INFO_STREAM("Size of internal_transforms: "<<utils.camera_internal_transforms_.size());
  for (int k=0; k<utils.calibrated_transforms_.size(); k++ )
  {
    utils.calibrated_transforms_[k]=utils.calibrated_transforms_[k]*utils.camera_internal_transforms_[k];
  }
  ROS_INFO_STREAM("Target frame1: "<<utils.target_frame_[0]);
  ROS_INFO_STREAM("World frame: "<<utils.world_frame_);
  try
  {
    utils.listener_.lookupTransform(utils.world_frame_,utils.target_frame_[0], ros::Time(0), temp_tf);
    utils.points_to_world_transforms_.push_back(temp_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  for (int k=0; k<utils.calibrated_transforms_.size(); k++ )
  {
    utils.calibrated_transforms_[k]=utils.points_to_world_transforms_[0]*utils.calibrated_transforms_[k];
  }
  for (int k=0; k<utils.broadcasters_.size(); k++ )
  {
    utils.broadcasters_[k].sendTransform(tf::StampedTransform(utils.calibrated_transforms_[k], ros::Time::now(),
                                                              utils.world_frame_, utils.camera_intermediate_frame_[k]));
  }
  ROS_INFO_STREAM("Camera pose(s) published");
  if (Cal_job.store())
  {
    ROS_INFO_STREAM("Calibration job optimization saved to file");
  }

}

void pblockToPose(industrial_extrinsic_cal::P_BLOCK optimized_input)
{
  double R[9];
  double aa[3];
  aa[0] = optimized_input[0];
  aa[1] = optimized_input[1];
  aa[2] = optimized_input[2];
  ceres::AngleAxisToRotationMatrix(aa, R);
  double rx = atan2(R[7], R[8]);
  double ry = atan2(-R[6], sqrt(R[7] * R[7] + R[8] * R[8]));
  double rz = atan2(R[3], R[0]);
  //double rx = atan2(R[5], R[8]);
  //double ry = atan2(-R[2], sqrt(R[5] * R[5] + R[8] * R[8]));
  //double rz = atan2(R[1], R[0]);
  Eigen::Matrix4f mod_matrix;

  double ix = -(optimized_input[3] * R[0] + optimized_input[4] * R[1] + optimized_input[5] * R[2]);
  double iy = -(optimized_input[3] * R[3] + optimized_input[4] * R[4] + optimized_input[5] * R[5]);
  double iz = -(optimized_input[3] * R[6] + optimized_input[4] * R[7] + optimized_input[5] * R[8]);

  tf::Quaternion tf_quater;
  tf::Matrix3x3 tf_mod_matrix;
  tf_mod_matrix.setRPY(rx, ry, rz);
  tf_mod_matrix.getRotation(tf_quater);
  double roll, pitch, yaw;
  tf_mod_matrix.getRPY(roll, pitch, yaw);
  tf::Vector3 tf_transl;
  //tf_transl.setValue(optimized_input[3], optimized_input[4], optimized_input[5]);
  tf_transl.setValue(ix, iy, iz);
  ROS_INFO_STREAM("Origin: "<< tf_transl.x()<<", " <<tf_transl.y()<<", "<<tf_transl.z());
  ROS_INFO_STREAM("Roll, pitch, yaw: "<< roll <<", " <<pitch<<", "<<yaw);
  tf::Transform tf_model;
  tf_model.setRotation(tf_quater);
  tf_model.setOrigin(tf_transl);
  transforms.push_back(tf_model);
  //tf_models.push_back(tf_model);
  geometry_msgs::Pose pose_msg;
  tf::poseTFToMsg(tf_model, pose_msg);
  pose_msgs.push_back(pose_msg);
}

void generateMessages(std::vector<geometry_msgs::Pose> poses)
{

  geometry_msgs::PoseStamped pose1_msg, pose2_msg, pose3_msg, pose4_msg;
  pose1_msg.pose.orientation = poses[0].orientation;
  pose1_msg.pose.position =poses[0].position;
  pose1_msg.header.frame_id = "/world_frame";
  pose1_msg.header.stamp=ros::Time::now();

  pose2_msg.pose.orientation = poses[1].orientation;
  pose2_msg.pose.position = poses[1].position;
  pose2_msg.header.frame_id = "/world_frame";
  pose2_msg.header.stamp=ros::Time::now();
  transform_pub1.publish(pose1_msg);
  transform_pub2.publish(pose2_msg);

  pose3_msg.pose.orientation = poses[2].orientation;
  pose3_msg.pose.position = poses[2].position;
  pose3_msg.header.frame_id = "/world_frame";
  pose3_msg.header.stamp=ros::Time::now();

  pose4_msg.pose.orientation =poses[3].orientation;
  pose4_msg.pose.position = poses[3].position;
  pose4_msg.header.frame_id = "/world_frame";
  pose4_msg.header.stamp=ros::Time::now();
  transform_pub3.publish(pose3_msg);
  transform_pub4.publish(pose4_msg);
}
// angle axis to homogeneous transform
void print_AAtoH(double &x, double &y, double &z, double &tx, double &ty, double &tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[3], R[6], tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[1], R[4], R[7], ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[2], R[5], R[8], tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}
// angle axis to homogeneous transform inverted
void print_AAToHI(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  double ix = -(tx * R[0] + ty * R[1] + tz * R[2]);
  double iy = -(tx * R[3] + ty * R[4] + tz * R[5]);
  double iz = -(tx * R[6] + ty * R[7] + tz * R[8]);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[1], R[2], ix);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[3], R[4], R[5], iy);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[6], R[7], R[8], iz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}
