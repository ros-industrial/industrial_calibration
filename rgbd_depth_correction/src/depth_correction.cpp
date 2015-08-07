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


namespace rgbd_depth_correction{

class DepthCorrectionNodelet : public nodelet::Nodelet
{
private:

  bool depth_exp_;
  double d1_, d2_;

  int version_;
  std::vector<double> depth_correction_;
  pcl::PointCloud<pcl::PointXYZRGB> correction_cloud_;

  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  ros::ServiceServer depth_change_;

  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

  bool readYamlFile(const std::string & pathway, const std::string & yaml_file);

  void loadVersionOne(const YAML::Node& doc, const std::string& file);
  void correctionVersionOne(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

public:

  bool setEnableDepth(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
  {
    depth_exp_ = !depth_exp_;
    ROS_INFO_STREAM("Using depth exponential correction: " << depth_exp_);
    return true;
  }

  virtual void onInit()
  {
    depth_exp_ = true;
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle priv_nh = getMTPrivateNodeHandle();

    // get parameters for services, topics, and filename
    std::string filename, filepath;
    if(!priv_nh.getParam("filename", filename) || !priv_nh.getParam("filepath", filepath))
    {
      ROS_ERROR("File name and/or pathway not provided.  Closing depth correction node.");
      return;
    }

    ROS_INFO_STREAM("Reading in yaml file " << filepath << filename);

    if(!readYamlFile(filepath, filename))
    {
      ROS_ERROR("Error reading YAML config file.  Closing depth correction node.");
      return;
    }

    ROS_INFO("Done reading yaml file");

    pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("out_cloud",1);
    pcl_sub_ = nh.subscribe("in_cloud", 1, &DepthCorrectionNodelet::pointcloudCallback, this);

    depth_change_ = nh.advertiseService("change_depth_factor", &DepthCorrectionNodelet::setEnableDepth, this);

  }
};

void DepthCorrectionNodelet::pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  switch(version_)
  {
    case 1:
      correctionVersionOne(cloud);
      break;
    default:
      ROS_ERROR_STREAM_THROTTLE(120, "Depth calibration file version does not match any known versions.  Not performing depth correction");
      break;
  }
}

bool DepthCorrectionNodelet::readYamlFile(const std::string &pathway, const std::string &yaml_file)
{
  bool rtn = true;
  std::string file;
  file = pathway + yaml_file + ".yaml";
  std::ifstream fin(file.c_str());
  if( fin.is_open())
  {
    try
    {
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);

      if(const YAML::Node *pName = doc.FindValue("version"))
      {
        *pName >> version_;
      }

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
    catch(YAML::ParserException& e)
    {
      ROS_ERROR_STREAM("Yaml parser exception: " << e.what());
      rtn = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to read in yaml file: " << yaml_file);
    rtn = false;
  }

  return rtn;
}

void DepthCorrectionNodelet::loadVersionOne(const YAML::Node &doc, const std::string &file)
{
  if(const YAML::Node *pName = doc.FindValue("d1"))
  {
    *pName >> d1_;
  }
  if(const YAML::Node *pName = doc.FindValue("d2"))
  {
    *pName >> d2_;
  }

  std::string pcd_file;
  pcd_file = file + ".pcd";

  ROS_INFO_STREAM("loading pcd " << pcd_file);
  pcl::io::loadPCDFile(pcd_file, correction_cloud_);
}

void DepthCorrectionNodelet::correctionVersionOne(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> corrected_cloud = *cloud;
  if(correction_cloud_.points.size() == cloud->points.size())
  {
    for(int i = 0; i < corrected_cloud.points.size(); ++i)
    {
      if(isnan(corrected_cloud.points.at(i).x) || isnan(correction_cloud_.points.at(i).z))
      {
        continue;
      }

      if(depth_exp_)
      {
        corrected_cloud.points.at(i).z += correction_cloud_.points.at(i).z * exp(d1_ + d2_ * corrected_cloud.points.at(i).z);
      }
      else
      {
        corrected_cloud.points.at(i).z += correction_cloud_.points.at(i).z;
      }
    }
  }

  pcl_pub_.publish(corrected_cloud);
}

PLUGINLIB_DECLARE_CLASS(rgbd_depth_correction, DepthCorrectionNodelet, rgbd_depth_correction::DepthCorrectionNodelet, nodelet::Nodelet);
}
