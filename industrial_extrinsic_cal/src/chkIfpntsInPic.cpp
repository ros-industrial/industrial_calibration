//This program checks to see if the camera can see the target at different poses.
//Assumes a flat rectangular target.

#include <iostream>
#include <vector>
#include <industrial_extrinsic_cal/conical_pose_generator.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <industrial_extrinsic_cal/target.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>

using std::vector;
using std::string;
using industrial_extrinsic_cal::Target;

// local function prototypes
void imagePoint(Eigen::Vector3d TargetPoint, double fx, double fy, double cx, double cy, double &u, double &v,Eigen::Affine3d cameraPose);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");

  ros::NodeHandle n("~");
  double Tx, Ty, Tz;
  //TODO take in cordinates of dots, focal lengths, camera resolution, camera principle points

  int image_width;
  int image_height;
  double fy, fx, cx, cy;
  int numberOfStopsForPhotos= 10;
  double poseRadius = 3;
  double poseHeight = 3;
  int num_poses = numberOfStopsForPhotos*3+1;

  string targetFile = "Ltarget_file";
  //int myYamlDefinedTargets;

  // supposed to get the the cordinates of the dots

  vector<boost::shared_ptr<Target>>  myYamlDefinedTargets;
  /*
  parseTargets(targetFile , myYamlDefinedTargets);

   Target *Mytarget = (Target *) &myYamlDefinedTargets[0];

  double xMax,yMax,xMin,yMin;
  xMax = Mytarget->pts_[0].x;
  xMin = Mytarget->pts_[0].x;
  yMax = Mytarget->pts_[0].y;
  yMin = Mytarget->pts_[0].y;

  for(int i=0;i<Mytarget->num_points_;i++){
    if(Mytarget->pts_[i].x > xMax) xMax = Mytarget->pts_[i].x;
    if(Mytarget->pts_[i].y > yMax) yMax = Mytarget->pts_[i].y;
    if(Mytarget->pts_[i].x < xMin) xMin = Mytarget->pts_[i].x;
    if(Mytarget->pts_[i].y < yMin) xMin = Mytarget->pts_[i].y;
  }

  Eigen::Vector3d corner_points[4];

  corner_points[0] = Eigen::Vector3d(xMin,yMin,0);
  corner_points[1] = Eigen::Vector3d(xMax,yMin,0);
  corner_points[2] = Eigen::Vector3d(xMin,yMax,0);
  corner_points[3] = Eigen::Vector3d(xMax,yMax,0);
*/

  // creates poses. makes numberOfStopsForPhotos*3+1 different poses stored in "frames

  EigenSTL::vector_Affine3d cameraPoses = getConicalPoses(numberOfStopsForPhotos, poseHeight, poseRadius);

   ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("topic", 1, true);

  geometry_msgs::PoseArray msg;
  msg.header.frame_id = "target";
  msg.header.stamp = ros::Time::now();

  for(int i=0; i<cameraPoses.size(); i++ )
  {
    geometry_msgs::Pose pose;

    tf::poseEigenToMsg(cameraPoses[i], pose);

    msg.poses.push_back(pose);
  }

  pub.publish(msg);

  ros::spin();


//  vector<Eigen::Affine3d> accepted_poses;
//  for(int j=0;j<num_poses;j++){
//    bool accept_pose = true;
//    for(int i=0;i<4;i++){
//      double u,v;
//      Eigen::Affine3d CP = cameraPoses[i];
//      imagePoint(corner_points[i], fx, fy, cx, cy, u, v,CP);
//      if(u<0 || u>=image_width || v<0 || v>= image_height ) {
//        accept_pose = false;
//      }
//    }
//    if(accept_pose){
//      accepted_poses.push_back(cameraPoses[j]);
//    }
//  }
}


// Transforms a point into the camera frame then projects the new point into the 2d screen
// TODO fix syntax for imagePoint

void imagePoint(Eigen::Vector3d TargetPoint , double fx, double fy, double cx, double cy, double &u, double &v,Eigen::Affine3d cameraPose )
{
 Eigen::Vector3d cp;
 cp = cameraPose.inverse() * TargetPoint;
 u = fx*(cp.x()/cp.z()) + cx;
 v = fy*(cp.y()/cp.z()) + cy;
}

