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
int addingFactorial(int lastAdded);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  sleep(2);
  ros::NodeHandle n("~");
  double Tx, Ty, Tz;
  //TODO take in cordinates of dots, focal lengths, camera resolution, camera principle pointsd

  int image_width =  10;
  int image_height = 10;
  double fy, fx, cx, cy;
  int numberOfStopsForPhotos= 10;
  double poseHeight = 5;
  double angleOfCone = M_PI/4;
  int num_poses = numberOfStopsForPhotos*3+1;
  int amountOfRings;
  double center_Of_TargetX;
  double center_Of_TargetY;



  // supposed to get the the cordinates of the dots
  // TODO read in target file

  string targetFile = "/home/lawrencelewis/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/ical_srv_target.yaml";
  vector<boost::shared_ptr<Target>>  myYamlDefinedTargets;
  if(!parseTargets(targetFile , myYamlDefinedTargets))
{
    ROS_ERROR("file name bad");
    return 0;
  }
   boost::shared_ptr<Target> Mytarget = myYamlDefinedTargets[0];

  double xMax,yMax,xMin,yMin;
ROS_INFO("target name = %s num_points = %d size of pts_ = %d",Mytarget->target_name_,Mytarget->num_points_,(int)Mytarget->pts_.size());
  xMax = Mytarget->pts_[0].x;
  xMin = Mytarget->pts_[0].x;
  yMax = Mytarget->pts_[0].y;
  yMin = Mytarget->pts_[0].y;
ROS_INFO("Lawrence");
  for(int i=0;i<Mytarget->num_points_;i++){
    if(Mytarget->pts_[i].x > xMax) xMax = Mytarget->pts_[i].x;
    if(Mytarget->pts_[i].y > yMax) yMax = Mytarget->pts_[i].y;
    if(Mytarget->pts_[i].x < xMin) xMin = Mytarget->pts_[i].x;
    if(Mytarget->pts_[i].y < yMin) yMin = Mytarget->pts_[i].y;
  }
ROS_INFO("Lawrence");
  Eigen::Vector3d corner_points[4];

  corner_points[0] = Eigen::Vector3d(xMin,yMin,0);
  corner_points[1] = Eigen::Vector3d(xMax,yMin,0);
  corner_points[2] = Eigen::Vector3d(xMin,yMax,0);
  corner_points[3] = Eigen::Vector3d(xMax,yMax,0);

//   Eigen::Vector3d corner_points[4];

//  corner_points[0] = Eigen::Vector3d(0,0,0);
//  corner_points[1] = Eigen::Vector3d(7,0,0);
//  corner_points[2] = Eigen::Vector3d(0,7,0);
//  corner_points[3] = Eigen::Vector3d(7,7,0);

  // finds the middle of the rectangular target
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
    {
      if(corner_points[i][0] != corner_points[j][0])
      {
        center_Of_TargetX = (corner_points[i][0] + corner_points[j][0]) / 2;
      }
      if(corner_points[i][1] != corner_points[j][1])
      {
        center_Of_TargetY = (corner_points[i][1] + corner_points[j][1]) / 2;
      }
    }
  }


  // ////////////////////////////////////////////////////////////////////////////////////////////////
  // creates camera transforms in the shape of a cone


  geometry_msgs::PoseArray msg;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("topic", 1, true);
  // radius of the cone = (poseHeight/ std::tan(angleOfCone))
  int extra_Counter = 0;
  for(int j=1; j<=poseHeight; j++)
  {
    for(double l = (std::tan(angleOfCone)) ; l<=(std::tan(angleOfCone) * j ); l = l+(std::tan(angleOfCone)))
    {
      EigenSTL::vector_Affine3d cameraPoses = getConicalPoses(numberOfStopsForPhotos, j, l);
      msg.header.frame_id = "target";
      msg.header.stamp = ros::Time::now();
      extra_Counter ++;
      for(int i=0; i<cameraPoses.size(); i++ )
      {
        geometry_msgs::Pose pose;
        //cameraPoses[i]
        tf::poseEigenToMsg(cameraPoses[i], pose);
        ROS_INFO("adding a pose");
        pose.position.x = pose.position.x + center_Of_TargetX;

        pose.position.y = pose.position.y + center_Of_TargetY;

        msg.poses.push_back(pose);

      }
    }
  }
  geometry_msgs::Pose pnt1,pnt2,pnt3,pnt4;
  pnt1.position.x = corner_points[0][0];
  pnt1.position.y = corner_points[0][1];
  pnt1.position.z = corner_points[0][2];
  pnt1.orientation.w = 1.0;
  pnt1.orientation.x = 0.0;
  pnt1.orientation.y = 0.0;
  pnt1.orientation.z = 0.0;

  pnt2.position.x = corner_points[1][0];
  pnt2.position.y = corner_points[1][1];
  pnt2.position.z = corner_points[1][2];
  pnt2.orientation.w = 1.0;
  pnt2.orientation.x = 0.0;
  pnt2.orientation.y = 0.0;
  pnt2.orientation.z = 0.0;

  pnt3.position.x = corner_points[2][0];
  pnt3.position.y = corner_points[2][1];
  pnt3.position.z = corner_points[2][2];
  pnt3.orientation.w = 1.0;
  pnt3.orientation.x = 0.0;
  pnt3.orientation.y = 0.0;
  pnt3.orientation.z = 0.0;

  pnt4.position.x = corner_points[3][0];
  pnt4.position.y = corner_points[3][1];
  pnt4.position.z = corner_points[3][2];
  pnt4.orientation.w = 1.0;
  pnt4.orientation.x = 0.0;
  pnt4.orientation.y = 0.0;
  pnt4.orientation.z = 0.0;

  msg.poses.push_back(pnt1);
  msg.poses.push_back(pnt2);
  msg.poses.push_back(pnt3);
  msg.poses.push_back(pnt4);

  // ////////////////////////////////////////////////////////////////////////////////////////////////

  // convert the messages back into poses
  EigenSTL::vector_Affine3d AllcameraPoses(msg.poses.size());
  geometry_msgs::PoseArray msg2;
  for(int i=0; i< msg.poses.size(); i++)
  {
    tf::poseMsgToEigen(msg.poses[i],AllcameraPoses[i]);
  }

  int num_created_poses = AllcameraPoses.size();
  vector<Eigen::Affine3d> accepted_poses;
  for(int j=0;j<num_created_poses;j++)
  {
      bool accept_pose = true;
      for(int i=0;i<4;i++)
      {
          double u=0;
          double v=0;
          imagePoint(corner_points[i], fx, fy, cx, cy, u, v,AllcameraPoses[j]);
          if(u<0 || u>=image_width || v<0 || v>= image_height )
          {
            ROS_ERROR("u,v = %lf %lf but image size = %d %d",u,v,image_width,image_height);
            accept_pose = false;
          }
      }
      if(accept_pose)
      {
          geometry_msgs::Pose pose;
          ROS_INFO("adding an accepted pose");
          tf::poseEigenToMsg(AllcameraPoses[j], pose);
          msg2.poses.push_back(pose);
      }
  }

  ros::Publisher pub2 = n.advertise<geometry_msgs::PoseArray>("topic2", 1, true);
  msg2.header.frame_id = "target2";
  msg2.header.stamp = ros::Time::now();

//  for(int i; i<accepted_poses.size(); i++)
//  {


//    geometry_msgs::Pose pose;
//    tf::poseEigenToMsg(accepted_poses[i],pose);
//    msg2.poses.push_back(pose);
//  }


  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    pub2.publish(msg2);
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

}



// Transforms a point into the camera frame then projects the new point into the 2d screen
void imagePoint(Eigen::Vector3d TargetPoint , double fx, double fy, double cx, double cy, double &u, double &v,Eigen::Affine3d cameraPose )
{
 Eigen::Vector3d cp;
 cp = cameraPose.inverse() * TargetPoint;
 double divider = cp.z();
 if(divider == 0) divider = 1.0;
 u = fx*(cp.x()/cp.z()) + cx;
 v = fy*(cp.y()/cp.z()) + cy;
}
int addingFactorial(int lastAdded)
{
    int sumation = 0;
    while(lastAdded>0)
    {
       sumation = sumation +lastAdded;
       lastAdded = lastAdded-1;
    }
    return sumation;
}
int findingMidpoint(int pointOne, int pointTwo)
{
  int midpoint;
  return midpoint;
}

