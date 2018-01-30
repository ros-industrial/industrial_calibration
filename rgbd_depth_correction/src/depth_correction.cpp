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

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <yaml-cpp/yaml.h>

#include "pluginlib/class_list_macros.h"
#include <nodelet/nodelet.h>

#include <industrial_extrinsic_cal/yaml_utils.h>

namespace rgbd_depth_correction
{
class DepthCorrectionNodelet : public nodelet::Nodelet
{
private:
  bool use_depth_exp_; /**< @brief Flag to determine whether to use the depth coefficients or not */
  double d1_, d2_;     /**< @brief The depth coefficients */

  int version_;                                     /**< @brief The version number found in the YAML file */
  pcl::PointCloud<pcl::PointXYZ> correction_cloud_; /**< @brief The depth correction point cloud containing the depth
                                                       correction values */

  ros::Subscriber pcl_sub_; /**< @brief PCL point cloud subscriber */
  ros::Publisher pcl_pub_;  /**< @brief PCL point cloud publisher for the corrected point cloud */
  ros::ServiceServer
      depth_change_; /**< @brief The service server for changing whether to use the depth coefficients or not */

  /**
     * @brief PCL raw point cloud subscriber callback
     *
     * @param[in] cloud Latest point cloud received
     */
  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

  /**
     * @brief Give a pathway and file name, reads the version number and loads the appropriate depth correction
   * parameters
     *
     * @param[in] pathway The pathway to the file to be read
     * @param[in] yaml_file The name of the file to be read
     * @return True if the YAML file was read correctly, false if there was an error or the file is not of the correct
   * version
     */
  bool readYamlFile(const std::string& pathway, const std::string& yaml_file);

  /**
     * @brief Reads and loads the parameters for depth correction version one
     *
     * @param[in] doc The YAML document to read from
     * @param[in] file The file name used to the open similarly named point cloud depth correction file
     */
  void loadVersionOne(const YAML::Node& doc, const std::string& file);
  /**
     * @brief Peforms point cloud depth correction using the version one parameters and equations, then republishes the
   * corrected cloud
     *
     * @param[in] cloud The point cloud to be corrected and republished
     */
  void correctionVersionOne(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

public:
  /**
     * @brief For debug testing, turns off/on the use of the depth correction coefficients to see how accurate the
   * results are
     *
     * @param[in] request Empty
     * @param[out] response Empty
     * @return always returns true
     */
  bool setEnableDepth(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    use_depth_exp_ = !use_depth_exp_;
    ROS_INFO_STREAM("Using depth exponential correction: " << use_depth_exp_);
    return true;
  }

  /**
     * @brief On startup of nodelet, reads the YAML config file and starts the publishers/subscribers
     */
  virtual void onInit()
  {
    use_depth_exp_ = true;
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle priv_nh = getMTPrivateNodeHandle();

    std::string filename, filepath;

    if (!priv_nh.getParam("filepath", filepath))
    {
      ROS_ERROR("File pathway not provided.  Closing depth correction node.");
      return;
    }

    // Check for filename argument first, use 'get_serial' service if argument is not provided
    if (priv_nh.getParam("filename", filename))
    {
      ROS_INFO("Using provided camera file name argument '%s' for depth correction", filename.c_str());
    }
    else
    {
      ROS_ERROR("Cannot open configuration files because parameter 'filename' not provided. Closing depth correction "
                "node.");
      return;
    }

    // get parameters for services, topics, and filename

    ROS_INFO_STREAM("Reading in yaml file " << filepath << filename << ".yaml");

    if (!readYamlFile(filepath, filename))
    {
      ROS_ERROR("Error reading YAML config file.  Closing depth correction node.");
      return;
    }

    ROS_INFO("Done reading yaml file");

    pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("out_cloud", 1);
    pcl_sub_ = nh.subscribe("in_cloud", 1, &DepthCorrectionNodelet::pointcloudCallback, this);

    depth_change_ = nh.advertiseService("change_depth_factor", &DepthCorrectionNodelet::setEnableDepth, this);
  }
};

void DepthCorrectionNodelet::pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  switch (version_)
  {
    case 1:
      correctionVersionOne(cloud);
      break;
    default:
      ROS_ERROR_STREAM_THROTTLE(120, "Depth calibration file version does not match any known versions.  Not "
                                     "performing depth correction");
      break;
  }
}

bool DepthCorrectionNodelet::readYamlFile(const std::string& pathway, const std::string& yaml_file)
{
  bool rtn = true;
  std::string file;
  file = pathway + yaml_file + ".yaml";
  YAML::Node doc;
  if (!industrial_extrinsic_cal::yamlNodeFromFileName(file, doc))
  {
    ROS_ERROR("could not open yaml file %s", file.c_str());
    rtn = false;
  }
  else if (!industrial_extrinsic_cal::parseInt(doc, "version", version_))
  {
    ROS_ERROR("Yaml file did not contain depth correction version information");
    rtn = false;
  }
  else
  {
    switch (version_)
    {
      case 1:
        loadVersionOne(doc, pathway + yaml_file);
        break;
      default:
        ROS_ERROR("Version in YAML file did not match any known versions");
        rtn = false;
        break;
    }
  }
  return rtn;
}

void DepthCorrectionNodelet::loadVersionOne(const YAML::Node& doc, const std::string& file)
{
  if (!industrial_extrinsic_cal::parseDouble(doc, "d1", d1_))
  {
    ROS_ERROR("Yaml file did not contain depth correction parameter d1, setting to zero");
    d1_ = 0;
  }
  if (!industrial_extrinsic_cal::parseDouble(doc, "d2", d2_))
  {
    ROS_ERROR("Yaml file did not contain depth correction parameter d2, setting to zero");
    d2_ = 0;
  }

  std::string pcd_file;
  pcd_file = file + ".pcd";

  ROS_INFO_STREAM("loading pcd " << pcd_file);
  pcl::io::loadPCDFile(pcd_file, correction_cloud_);
}

void DepthCorrectionNodelet::correctionVersionOne(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ> corrected_cloud = *cloud;
  if (correction_cloud_.points.size() == cloud->points.size())
  {
    for (int i = 0; i < corrected_cloud.points.size(); ++i)
    {
      if (std::isnan(corrected_cloud.points.at(i).x) || std::isnan(correction_cloud_.points.at(i).z))
      {
        continue;
      }

      if (use_depth_exp_)
      {
        corrected_cloud.points.at(i).z +=
            correction_cloud_.points.at(i).z * exp(d1_ + d2_ * corrected_cloud.points.at(i).z);
      }
      else
      {
        corrected_cloud.points.at(i).z += correction_cloud_.points.at(i).z;
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(30, "Depth correction cloud size and input point cloud size do not match.  Not "
                                  "performing depth correction");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr published_cloud = corrected_cloud.makeShared();
  pcl_pub_.publish(published_cloud);
}

PLUGINLIB_DECLARE_CLASS(rgbd_depth_correction, DepthCorrectionNodelet, rgbd_depth_correction::DepthCorrectionNodelet,
                        nodelet::Nodelet);
}
