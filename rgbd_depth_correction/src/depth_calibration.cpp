/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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

#include <depth_calibration/depth_calibration.h>
#include <target_finder/target_locator.h>
#include <boost/thread/locks.hpp>
#include <sstream>

#include <tf/tf.h>
#include <geometry_msgs/Transform.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

const unsigned int DepthCalibrator::VERSION_NUMBER_ = 1;

DepthCalibrator::DepthCalibrator(ros::NodeHandle& nh)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  // get parameters for services, topics, and filename
  if (!pnh.getParam("filename", filename_))
  {
    ROS_WARN("Depth calibration file name not given for saving calibration results.  Defaulting to 'camera'.");
    filename_ = "camera";
  }
  if (!pnh.getParam("filepath", filepath_))
  {
    ROS_WARN("Depth calibration file path not given for saving calibration results.  Results will not be saved.  "
             "Please provide path and re-execute to save results");
    save_data_ = false;
  }
  else
  {
    save_data_ = true;
  }

  std_dev_error_ = 0.1;
  depth_error_threshold_ = 0.2;

  double x, y, z, w;

  pnh.param<double>("target_pos_x", x, 0.0);
  pnh.param<double>("target_pos_y", y, 0.0);
  pnh.param<double>("target_pos_z", z, 0.0);

  target_initial_pose_.position.x = x;
  target_initial_pose_.position.y = y;
  target_initial_pose_.position.z = z;

  pnh.param<double>("target_orn_x", x, 0.0);
  pnh.param<double>("target_orn_y", y, 0.0);
  pnh.param<double>("target_orn_z", z, 0.0);
  pnh.param<double>("target_orn_w", w, 1.0);

  target_initial_pose_.orientation.x = x;
  target_initial_pose_.orientation.y = y;
  target_initial_pose_.orientation.z = z;
  target_initial_pose_.orientation.w = w;
  ROS_WARN("found target at xyz: (%.2f, %.2f, %.2f) wxyz: (%.2f, %.2f, %.2f, %.2f)", target_initial_pose_.position.x,
           target_initial_pose_.position.y, target_initial_pose_.position.z, target_initial_pose_.orientation.w,
           target_initial_pose_.orientation.x, target_initial_pose_.orientation.y, target_initial_pose_.orientation.z);

  pnh.param<int>("num_views", num_views_, 30);
  pnh.param<int>("num_attempts", num_attempts_, 10);
  pnh.param<int>("point_cloud_history", num_point_clouds_, 30);

  // Create Subscribers and Services
  calibrate_depth_ = nh_.advertiseService("depth_calibration", &DepthCalibrator::calibrateCameraDepth, this);
  calibrate_pixel_depth_ =
      nh_.advertiseService("pixel_depth_calibration", &DepthCalibrator::calibrateCameraPixelDepth, this);
  set_store_cloud_ = nh_.advertiseService("store_cloud", &DepthCalibrator::setStoreCloud, this);
  get_target_pose_ = nh.serviceClient<target_finder::target_locator>("TargetLocateService");

  this->point_cloud_sub_ =
      boost::shared_ptr<PointCloudSubscriberType>(new PointCloudSubscriberType(nh, "depth_points", 1));
  this->image_sub_ = boost::shared_ptr<ImageSubscriberType>(new ImageSubscriberType(nh, "image", 1));
  synchronizer_ = boost::shared_ptr<SynchronizerType>(
      new SynchronizerType(PolicyType(10), *this->point_cloud_sub_, *this->image_sub_));
  synchronizer_->registerCallback(boost::bind(&DepthCalibrator::updateInputData, this, _1, _2));
}

void DepthCalibrator::storeCalibration(const std::string& yaml_file, const double dp[2])
{
  ROS_INFO_STREAM("writing calibration data to file " << yaml_file);
  std::ofstream fout(yaml_file.c_str());
  YAML::Emitter yaml_emitter;
  yaml_emitter << YAML::Comment("rgbd_depth_correction calibration file");
  yaml_emitter << YAML::BeginMap;

  yaml_emitter << YAML::Key << "version" << YAML::Value << VERSION_NUMBER_;
  yaml_emitter << YAML::Key << "d1" << YAML::Value << dp[0];
  yaml_emitter << YAML::Key << "d2" << YAML::Value << dp[1];

  yaml_emitter << YAML::EndMap;
  fout << yaml_emitter.c_str();
  fout.close();
}

bool DepthCalibrator::calibrateCameraDepth(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);

  if (saved_clouds_.size() > 0)
  {
    if (!(saved_clouds_[0].points.size() == correction_cloud_.points.size()))
    {
      ROS_ERROR("Point cloud pixel depth correction cloud size (%lu) not the same size as saved cloud size (%lu). "
                "Aborting depth calibration.",
                correction_cloud_.points.size(), saved_clouds_[0].points.size());
      return false;
    }
  }
  else
  {
    ROS_ERROR("No point cloud data available.  Aborting depth calibration");
    return false;
  }

  ROS_INFO("Creating optimization problem to solve for depth coefficients");
  ceres::Problem problem;
  double dp[2];
  dp[0] = dp[1] = 0;

  // Add each point in all point clouds as a new cost
  for (int i = 0; i < saved_clouds_[0].points.size(); ++i)
  {
    for (int j = 0; j < saved_clouds_.size(); ++j)
    {
      if (std::isnan(saved_clouds_[j].points.at(i).x) || saved_clouds_[j].points.at(i).z == 0)
      {
        continue;
      }
      std::vector<double> eq;
      eq = plane_equations_[j];
      ceres::CostFunction* cost_function = DepthError::Create(
          eq[0], eq[1], eq[2], eq[3], correction_cloud_.points.at(i).z, saved_clouds_[j].points.at(i));
      problem.AddResidualBlock(cost_function, NULL, dp);
    }
  }

  // Create Ceres problem to optimize depth correction coefficients
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  ceres::Solve(options, &problem, &summary);

  ROS_INFO("depth calibration resulting parameters: d1: %.3f, d2: %.3f ", dp[0], dp[1]);
  ROS_INFO("Finished calculating in %.2f seconds with a residual error of: %.3f (and num residuals: %d)",
           summary.total_time_in_seconds, summary.final_cost, summary.num_residuals);

  // Store distortion coefficients in YAML file
  if (save_data_)
  {
    storeCalibration((filepath_ + "/" + filename_ + ".yaml"), dp);
  }
  saved_clouds_.clear();
  plane_equations_.clear();
  saved_images_.clear();
}

bool DepthCalibrator::findAveragePointCloud(pcl::PointCloud<pcl::PointXYZ>& final_cloud)
{
  // Store point clouds until the number of point clouds desired is reached, break if no new data is received after 1
  // second
  int rate = 100;
  std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > temp_clouds;
  ros::Rate sleep_rate(rate);
  while (temp_clouds.size() < num_point_clouds_)
  {
    ros::Time new_time = ros::Time::now();
    int count = 0;
    while (count < rate)
    {
      {
        boost::lock_guard<boost::mutex> lock(data_lock_);
        std_msgs::Header ros_cloud_header;
        pcl_conversions::fromPCL(last_cloud_.header, ros_cloud_header);
        final_cloud = last_cloud_;
        if (ros_cloud_header.stamp > new_time)
        {
          ROS_INFO("Got point cloud %lu of %d", (temp_clouds.size() + 1), num_point_clouds_);
          temp_clouds.push_back(last_cloud_);
          break;
        }
      }
      sleep_rate.sleep();
      ++count;
    }

    if (count == rate)
    {
      ROS_ERROR("No new point cloud data after 1 second");
      return false;
    }
  }

  final_cloud.points.clear();

  // Calculate average depth error for all pixels
  for (int i = 0; i < temp_clouds[0].points.size(); ++i)
  {
    // Find the average depth value for the current pixel point
    int count = 0;
    double x, y, z;
    pcl::PointXYZ pt;
    x = y = z = 0.0;
    for (int j = 0; j < temp_clouds.size(); ++j)
    {
      if (std::isnan(temp_clouds[j].points.at(i).x) || temp_clouds[j].points.at(i).z == 0)
      {
        continue;
      }

      ++count;
      x += temp_clouds[j].points.at(i).x;
      y += temp_clouds[j].points.at(i).y;
      z += temp_clouds[j].points.at(i).z;
    }

    if (count > 0)
    {
      x = x / double(count);
      y = y / double(count);
      z = z / double(count);

      pt.x = x;
      pt.y = y;
      pt.z = z;
      final_cloud.points.push_back(pt);
    }
    else
    {
      pcl::PointXYZ pt;
      pt.x = NAN;
      pt.y = NAN;
      pt.z = NAN;
      final_cloud.points.push_back(pt);
    }
  }
  ROS_WARN("done getting average cloud");
  return true;
}

bool DepthCalibrator::calibrateCameraPixelDepth(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // Find the target location
  std::vector<double> plane_eq;
  geometry_msgs::Pose target_pose;
  ROS_INFO("Attempting to find target average pose");
  if (!findAveragePlane(plane_eq, target_pose))
  {
    ROS_ERROR("Failed to get target pose.  Aborting depth calibration");
    return false;
  }
  ROS_INFO("Target average pose found");

  // Get average point cloud
  pcl::PointCloud<pcl::PointXYZ> avg_cloud;
  if (!findAveragePointCloud(avg_cloud))
  {
    ROS_ERROR("Failed to get average point cloud.  Aborting depth calibration");
    return false;
  }

  ROS_WARN("correction cloud");
  // Calculate the correction cloud depth values
  correction_cloud_.points.clear();
  correction_cloud_.header = avg_cloud.header;
  correction_cloud_.width = avg_cloud.width;
  correction_cloud_.height = avg_cloud.height;
  for (int j = 0; j < avg_cloud.points.size(); ++j)
  {
    if (std::isnan(avg_cloud.points.at(j).x) || avg_cloud.points.at(j).z == 0)
    {
      pcl::PointXYZ pt;
      pt.x = NAN;
      pt.y = NAN;
      pt.z = NAN;
      correction_cloud_.points.push_back(pt);
      continue;
    }
    // calculated (ideal) depth using plane equation (ax + by + cz + d = 0)
    double ideal_depth =
        (-plane_eq[3] - plane_eq[0] * avg_cloud.points.at(j).x - plane_eq[1] * avg_cloud.points.at(j).y) / plane_eq[2];

    // depth correction value is error between calculated depth and average depth for the given pixel
    double error = (ideal_depth - avg_cloud.points.at(j).z);

    if (fabs(error) > depth_error_threshold_)
    {
      ROS_WARN("depth error for pixel %d is too great (%.3f), setting to NAN", j, error);
      pcl::PointXYZ pt;
      pt.x = NAN;
      pt.y = NAN;
      pt.z = NAN;
      correction_cloud_.points.push_back(pt);
    }
    else
    {
      pcl::PointXYZ pt;
      pt.x = avg_cloud.points.at(j).x;
      pt.y = avg_cloud.points.at(j).y;
      pt.z = error;
      correction_cloud_.points.push_back(pt);
    }
  }

  // Iterate through depth correction cloud and replace all NaNs with average value of neighbors
  // Repeat until no NaNs remain
  bool done = false;
  while (!done)
  {
    int num_nan = 0;
    bool nan_found = false;
    for (int i = 0; i < correction_cloud_.points.size(); ++i)
    {
      // If value is NaN, find average of neighbors
      if (std::isnan(correction_cloud_.points.at(i).x))
      {
        ++num_nan;
        nan_found = true;
        double val = 0.0;
        int count = 0;

        if ((i + 1) % (correction_cloud_.width) > 0 ||
            i == 0)  // add point to the right except when at the far right side
        {
          if (std::isnan(correction_cloud_.points.at(i + 1).z) == 0)
          {
            val += correction_cloud_.points.at(i + 1).z;
            ++count;
          }
        }

        if (i % correction_cloud_.width > 0)  // add point to the left except when at the far left side
        {
          if (std::isnan(correction_cloud_.points.at(i - 1).z) == 0)
          {
            val += correction_cloud_.points.at(i - 1).z;
            ++count;
          }
        }

        if (i > (correction_cloud_.width - 1))  // add point to the top except when at the top row
        {
          if (std::isnan(correction_cloud_.points.at(i - correction_cloud_.width).z) == 0)
          {
            val += correction_cloud_.points.at(i - correction_cloud_.width).z;
            ++count;
          }
        }

        if ((i + 1) < correction_cloud_.height * correction_cloud_.width -
                          correction_cloud_.width)  // add point to the bottom except when at the bottom row
        {
          if (std::isnan(correction_cloud_.points.at(i + correction_cloud_.width).z) == 0)
          {
            val += correction_cloud_.points.at(i + correction_cloud_.width).z;
            ++count;
          }
        }

        if (count > 0)
        {
          pcl::PointXYZ pt;
          pt.x = 0;
          pt.y = 0;
          pt.z = val / double(count);
          correction_cloud_.points.at(i) = pt;
        }
      }
    }
    done = !nan_found;
  }

  correction_cloud_.is_dense = false;

  // Store depth correction results
  if (save_data_)
  {
    std::string out_file = filepath_ + "/" + filename_ + ".pcd";
    ROS_INFO_STREAM("Done generating depth correction point cloud.  Saving results to " << out_file);
    pcl::io::savePCDFile(out_file, correction_cloud_);
  }

  return true;
}

bool DepthCalibrator::setStoreCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  geometry_msgs::Pose target_pose;

  try
  {
    std::vector<double> plane_eq;
    if (!findAveragePlane(plane_eq, target_pose))
    {
      ROS_ERROR("Failed to get target pose.  Not storing depth data");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ> avg_cloud;
    if (!findAveragePointCloud(avg_cloud))
    {
      ROS_ERROR("Failed to get average point cloud.  Not storing depth data");
      return false;
    }
    else
    {
      plane_equations_.push_back(plane_eq);
      saved_target_poses_.push_back(target_pose);
      avg_cloud.is_dense = false;
      saved_clouds_.push_back(avg_cloud);
      cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
      saved_images_.push_back(bridge->image);

      ROS_INFO("Point cloud and image successfully collected");
    }
  }
  catch (...)
  {
    ROS_ERROR("Error attempting to store point cloud");
  }
  return true;
}

void DepthCalibrator::updateInputData(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                      const sensor_msgs::ImageConstPtr& image)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);

  sensor_msgs::PointCloud2 temp_cloud;
  last_image_ = *image;
  temp_cloud = *cloud;
  temp_cloud.is_dense = false;
  pcl::fromROSMsg(temp_cloud, last_cloud_);
}

bool DepthCalibrator::findAveragePlane(std::vector<double>& plane_eq, geometry_msgs::Pose& target_pose)
{
  bool rtn = false;

  plane_eq.clear();
  geometry_msgs::Pose temp_pose;
  std::vector<double> a, b, c, d;
  int error = 0;

  // Find the target multiple times or until the error limit is reached
  while (a.size() < num_views_ && error < num_attempts_)
  {
    if (!findTarget(10.0, temp_pose))
    {
      ++error;
      continue;
    }
    ROS_WARN("found target at xyz: (%.2f, %.2f, %.2f) wxyz: (%.2f, %.2f, %.2f, %.2f)", temp_pose.position.x,
             temp_pose.position.y, temp_pose.position.z, temp_pose.orientation.w, temp_pose.orientation.x,
             temp_pose.orientation.y, temp_pose.orientation.z);
    // Create plane equation from target pose
    tf::Transform transform;
    tf::poseMsgToTF(temp_pose, transform);

    // Get the z-axis to define the plane normal direction
    double pa, pb, pc;
    pa = transform.getBasis().getColumn(2)[0];
    pb = transform.getBasis().getColumn(2)[1];
    pc = transform.getBasis().getColumn(2)[2];

    a.push_back(pa);
    b.push_back(pb);
    c.push_back(pc);
    d.push_back(-pa * transform.getOrigin().x() - pb * transform.getOrigin().y() - pc * transform.getOrigin().z());
  }

  target_pose = temp_pose;

  if (error < num_attempts_)
  {
    // calculate the average plane equation parameters
    double avg_a, avg_b, avg_c, avg_d, std_a, std_b, std_c, std_d;
    avg_a = avg_b = avg_c = avg_d = std_a = std_b = std_c = std_d = 0;
    for (int i = 0; i < a.size(); ++i)
    {
      avg_a += a[i];
      avg_b += b[i];
      avg_c += c[i];
      avg_d += d[i];
    }
    avg_a = avg_a / a.size();
    avg_b = avg_b / b.size();
    avg_c = avg_c / c.size();
    avg_d = avg_d / d.size();

    for (int i = 0; i < a.size(); ++i)
    {
      std_a += pow(a[i] - avg_a, 2.0);
      std_b += pow(b[i] - avg_b, 2.0);
      std_c += pow(c[i] - avg_c, 2.0);
      std_d += pow(d[i] - avg_d, 2.0);
    }
    std_a = sqrt(std_a / (a.size() - 1));
    std_b = sqrt(std_a / (a.size() - 1));
    std_c = sqrt(std_a / (a.size() - 1));
    std_d = sqrt(std_a / (a.size() - 1));

    // If standard deviation is too large, return an error
    if (std_a < std_dev_error_ && std_b < std_dev_error_ && std_c < std_dev_error_ && std_d < std_dev_error_)
    {
      plane_eq.push_back(avg_a);
      plane_eq.push_back(avg_b);
      plane_eq.push_back(avg_c);
      plane_eq.push_back(avg_d);
      rtn = true;
    }
    else
    {
      ROS_ERROR("Standard deviation of target pose is too large (possible target motion)");
    }
  }
  else
  {
    ROS_ERROR("Error limit reached trying to find average target pose");
  }

  return rtn;
}

bool DepthCalibrator::findTarget(const double& final_cost, geometry_msgs::Pose& target_pose)
{
  bool rtn = true;
  // Get target pose
  target_finder::target_locatorRequest target_request;
  target_finder::target_locatorResponse target_response;
  target_request.allowable_cost_per_observation = 5000.0;
  target_request.roi.height = 480;
  target_request.roi.width = 640;
  target_request.roi.do_rectify = false;
  target_request.roi.x_offset = 0;
  target_request.roi.y_offset = 0;
  target_request.initial_pose = target_initial_pose_;

  if (!get_target_pose_.call(target_request, target_response))
  {
    ROS_ERROR("Failed to get target pose.");
    rtn = false;
  }
  else if (target_response.final_cost_per_observation > final_cost)
  {
    ROS_ERROR("Target pose error too large (%.3f).", target_response.final_cost_per_observation);
    rtn = false;
  }

  target_pose = target_response.final_pose;
  return rtn;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_calibration_service");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  DepthCalibrator calibrator(nh);

  ros::spin();

  return 0;
}
