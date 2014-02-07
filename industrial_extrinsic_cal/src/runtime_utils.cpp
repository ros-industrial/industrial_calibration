/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/runtime_utils.h>

namespace industrial_extrinsic_cal
{

tf::Transform ROSRuntimeUtils::pblockToPose(industrial_extrinsic_cal::P_BLOCK &optimized_input)
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
  tf::Transform transform_output;
  transform_output.setRotation(tf_quater);
  transform_output.setOrigin(tf_transl);
  return transform_output;
}
bool ROSRuntimeUtils::store_tf_broadcasters(std::string &package_path, std::string &file_name)
{
  std::string filepath = package_path+file_name;
  std::ofstream output_file(filepath.c_str(), std::ios::out);// | std::ios::app);
  if (output_file.is_open())
  {
    ROS_INFO_STREAM("Storing results in: "<<filepath);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file");
    return false;
  }
  output_file << "<launch>";
  //have calibrated transforms
  double roll, pitch, yaw, x_position, y_position, z_position;
  tf::Vector3 origin;
  tf::Matrix3x3 orientation;
  for (int i=0; i<calibrated_transforms_.size(); i++)
  {
    origin = calibrated_transforms_.at(i).getOrigin();
    x_position=origin.getX();
    y_position=origin.getY();
    z_position=origin.getZ();
    orientation = calibrated_transforms_.at(i).getBasis();
    orientation.getEulerYPR(yaw, pitch, roll);
    output_file<<"\n";
    output_file<<" <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"world_to_camera"<<i<<"\" args=\"";
    //tranform publisher launch files requires x y z yaw pitch roll
    output_file<<x_position<< ' '<<y_position<< ' '<<z_position<< ' '<<yaw<< ' '<<pitch<< ' '<<roll ;
    output_file<<" "<<world_frame_;
    output_file<<" "<<camera_intermediate_frame_[i];
    output_file<<" 100\" />";
  }
  output_file<<"\n";
  output_file << "</launch>";
  return true;
}
} // end of namespace
