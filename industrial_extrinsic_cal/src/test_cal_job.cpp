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
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

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
  ros::init(argc, argv, "my_node_name");
  ros::NodeHandle nh;
  transform_pub1= nh.advertise<geometry_msgs::PoseStamped>("camera1_pose", 1);
  transform_pub2= nh.advertise<geometry_msgs::PoseStamped>("camera2_pose", 1);
  transform_pub3= nh.advertise<geometry_msgs::PoseStamped>("target1_pose", 1);
  transform_pub4= nh.advertise<geometry_msgs::PoseStamped>("target2_pose", 1);
  static tf::TransformBroadcaster br_camera1, br_camera2;
  tf::Transform t_orig;
  t_orig.setOrigin( tf::Vector3(0,0, 0.0) );
  t_orig.setRotation( tf::Quaternion(0, 0, 0, 0) );
  br_camera1.sendTransform(tf::StampedTransform(t_orig, ros::Time::now(), "/world_frame", "/ns1_kinect_rgb_optical_frame"));
  br_camera2.sendTransform(tf::StampedTransform(t_orig, ros::Time::now(), "/world_frame", "/ns2_kinect_rgb_optical_frame"));


  string camera_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_camera_def.yaml");
  string target_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_target_def.yaml");
  string caljob_file_name(
      "/home/cgomez/ros/hydro/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/test1_caljob_def.yaml");
  CalibrationJob Cal_job(camera_file_name, target_file_name, caljob_file_name);
  //ROS_INFO_STREAM("hello world ");
  if (Cal_job.load())
  {
    ROS_INFO_STREAM("Calibration job (cal_job, target and camera) yaml parameters loaded.");
  }
  if (Cal_job.run())
  {
    ROS_INFO_STREAM("Calibration job observations and optimization complete");
  }
  //industrial_extrinsic_cal::P_BLOCK original_extrinsics= c_blocks.getStaticCameraParameterBlockExtrinsics("Asus1");
  std::vector<industrial_extrinsic_cal::P_BLOCK> opt_extrinsics = Cal_job.getExtrinsics();
  std::vector<industrial_extrinsic_cal::P_BLOCK> targets = Cal_job.getTargetPose();
  ROS_DEBUG_STREAM("Size of optimized_extrinsics_: "<<opt_extrinsics.size());
  ROS_DEBUG_STREAM("Size of targets_: "<<targets.size());

  industrial_extrinsic_cal::P_BLOCK optimized_extrinsics;
  std::vector<geometry_msgs::Pose> camera_pose_msgs;
  for (int k=0; k<opt_extrinsics.size(); k++ )
  {
    optimized_extrinsics=opt_extrinsics.at(k);
    ROS_INFO_STREAM("Optimized Camera "<<k);
    print_AAtoH(optimized_extrinsics[0], optimized_extrinsics[1], optimized_extrinsics[2],
                optimized_extrinsics[3], optimized_extrinsics[4], optimized_extrinsics[5]);
    print_AAToHI(optimized_extrinsics[0], optimized_extrinsics[1], optimized_extrinsics[2],
                optimized_extrinsics[3], optimized_extrinsics[4], optimized_extrinsics[5]);
    pblockToPose(optimized_extrinsics);
  }

  industrial_extrinsic_cal::P_BLOCK optimized_target;
  std::vector<geometry_msgs::Pose> target_pose_msgs;
  for (int k=0; k<targets.size(); k++ )
  {
    optimized_target=targets.at(k);
    ROS_INFO_STREAM("Optimized Target "<<k);
    print_AAtoH(optimized_target[0], optimized_target[1], optimized_target[2],
                optimized_target[3], optimized_target[4], optimized_target[5]);
    pblockToPose(optimized_target);
  }
  generateMessages(pose_msgs);

  //transforms.at(0).setRotation(tf::Quaternion(tf::Vector3(0,1,0), M_PI/2) );
  //transforms.at(1).setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI/2) );
  br_camera1.sendTransform(tf::StampedTransform(transforms[0], ros::Time::now(), "/world_frame", "/ns1_kinect_rgb_optical_frame"));
  br_camera2.sendTransform(tf::StampedTransform(transforms[1], ros::Time::now(), "/world_frame", "/ns2_kinect_rgb_optical_frame"));

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
// angle axis to homogeneous transform inverted
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
// angle axis to homogeneous transform
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
