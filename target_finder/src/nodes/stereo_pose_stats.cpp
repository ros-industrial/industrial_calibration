/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <target_finder/stereo_locator.h>
#include <tf/transform_broadcaster.h>
#include <industrial_extrinsic_cal/basic_types.h>

using std::string;
using std::vector;

class stereoStats
{
public:
  stereoStats(ros::NodeHandle nh) : nh_(nh)
  {
    client_ = nh_.serviceClient<target_finder::stereo_locator>("StereoLocatorService");
  }
  bool callTheService();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  target_finder::stereo_locator srv_;
};

bool stereoStats::callTheService()
{
  srv_.request.allowable_cost_per_observation = 3.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> qx;
  std::vector<double> qy;
  std::vector<double> qz;
  std::vector<double> qw;
	
  for(int i=0; i<30; i++){
    if (client_.call(srv_))
      {
	x.push_back(srv_.response.target_pose.position.x);
	y.push_back(srv_.response.target_pose.position.y);
	z.push_back(srv_.response.target_pose.position.z);
	qx.push_back(srv_.response.target_pose.orientation.x);
	qy.push_back(srv_.response.target_pose.orientation.y);
	qz.push_back(srv_.response.target_pose.orientation.z);
	qw.push_back(srv_.response.target_pose.orientation.w);
      }
  }
  double xmean=0;
  double ymean=0;
  double zmean=0;
  double qxmean=0;
  double qymean=0;
  double qzmean=0;
  double qwmean=0;
  int n = x.size();
  for(int i=0; i<n; i++){
    xmean+= x[i];
    ymean+= y[i];
    zmean+= z[i];
    qxmean+= qx[i];
    qymean+= qy[i];
    qzmean+= qz[i];
    qwmean+= qw[i];
  }
  xmean = xmean/n;
  ymean = ymean/n;
  zmean = zmean/n;
  qxmean = qxmean/n;
  qymean = qymean/n;
  qzmean = qzmean/n;
  qwmean = qwmean/n;
  double xs=0;
  double ys=0;
  double zs=0;
  double qxs=0;
  double qys=0;
  double qzs=0;
  double qws=0;
  for(int i=0; i<n; i++){
    xs+= (x[i]-xmean)*(x[i]-xmean);
    ys+= (y[i]-ymean)*(y[i]-ymean);
    zs+= (z[i]-zmean)*(z[i]-zmean);
    qxs+= (qx[i]-qxmean)*(qx[i]-qxmean);
    qys+= (qy[i]-qymean)*(qy[i]-qymean);
    qzs+= (qz[i]-qzmean)*(qz[i]-qzmean);
    qws+= (qw[i]-qwmean)*(qw[i]-qwmean);
  }
  xs  = sqrt(xs/(n-1));
  ys  = sqrt(ys/(n-1));
  zs  = sqrt(zs/(n-1));
  qxs = sqrt(qxs/(n-1));
  qys = sqrt(qys/(n-1));
  qzs = sqrt(qzs/(n-1));
  qws = sqrt(qws/(n-1));

  industrial_extrinsic_cal::Pose6d mean_pose;
  mean_pose.setOrigin(xmean,ymean,zmean);
  mean_pose.setQuaternion(qxmean, qymean, qzmean, qwmean);
  double ez,ey,ex;
  mean_pose.getEulerZYX(ez,ey,ex);

  ROS_INFO("mean  = %lf %lf %lf  %lf %lf %lf %lf euler: %lf %lf %lf", xmean, ymean, zmean, qxmean, qymean, qzmean, qwmean, ez, ey, ex);
  ROS_INFO("sigma = %lf %lf %lf  %lf %lf %lf %lf", xs, ys, zs, qxs, qys, qzs, qws);
  
  return (true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_pose_stats");
  ros::NodeHandle nh;
  stereoStats call_service(nh);
  ros::Rate loop_rate(.1);
  while (ros::ok())
  {
    call_service.callTheService();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
