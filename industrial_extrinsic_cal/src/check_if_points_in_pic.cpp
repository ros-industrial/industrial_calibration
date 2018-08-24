//This program checks to see if the camera can see the target at different poses.
//Assumes a flat rectangular target.

#include <vector>
#include <industrial_extrinsic_cal/conical_pose_generator.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <industrial_extrinsic_cal/target.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>

#include "applicable_pose_generator/pose_reachability_filter.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <industrial_extrinsic_cal/check_if_points_in_pic.h>

#include <industrial_extrinsic_cal/camera_yaml_parser.h>



using std::vector;
using std::string;
using industrial_extrinsic_cal::Target;
using YAML::Node;

// local function prototypes
int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle n("~");
  ros::NodeHandle pivnh("~");

  //loads all known values and parameters
  check_if_point_in_pic initialization(pivnh);

  initialization.center_point_of_target = initialization.finds_middle_of_target(initialization.center_Of_TargetX, initialization.center_Of_TargetY);

  // creates camera transforms in the shape of a cone
  ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("topic", 1, true);

  // radius of the cone = (poseHeight/ std::tan(angleOfCone))
  geometry_msgs::PoseArray msg = initialization.create_all_poses(initialization.poseHeight, initialization.spacing_in_z, initialization.angleOfCone, initialization.numberOfStopsForPhotos,  initialization.center_point_of_target );

  //creates rectanglular target in rviz
  initialization.create_rviz_target(msg);

  // convert the messages back into poses
  EigenSTL::vector_Affine3d AllcameraPoses(msg.poses.size());
  geometry_msgs::PoseArray msg2;
  for(int i=0; i< msg.poses.size(); i++)
  {
    tf::poseMsgToEigen(msg.poses[i],AllcameraPoses[i]);
  }

  // filters out unreachable and unseeable poses
  geometry_msgs::PoseArray filtered_msgs = initialization.pose_filters(msg2,initialization.tf_transform, initialization.image_width,initialization.image_height, AllcameraPoses, initialization.fx,  initialization.fy,  initialization.cx_,  initialization.cy_ );

  ros::Publisher pub2 = n.advertise<geometry_msgs::PoseArray>("topic2", 1, true);
  filtered_msgs.header.frame_id = "target";
  filtered_msgs.header.stamp = ros::Time::now();

  // publishes all transforms
  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    pub2.publish(filtered_msgs);
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
} //end of main

// Transforms a point into the camera frame then projects the new point into the 2d screen
void check_if_point_in_pic::imagePoint(Eigen::Vector3d TargetPoint , double fx, double fy, double cx, double cy, double &u, double &v,const Eigen::Affine3d &cameraPose )
{
 Eigen::Vector3d cp;
 cp = cameraPose.inverse() * TargetPoint;
 double divider = cp.z();
 if(divider <= .0001)
 {
   ROS_ERROR("Camera can't be here");
 }
 if(divider == 0) divider = 1.0;

 u = fx*(cp.x()/divider) + cx;
 v = fy*(cp.y()/divider) + cy;
}

int check_if_point_in_pic::addingFactorial(int lastAdded)
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

Eigen::Vector2d check_if_point_in_pic::finds_middle_of_target( double center_Of_TargetX, double center_Of_TargetY)
{
  Eigen::Vector2d center_point;
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
    {
      if(corner_points_[i][0] != corner_points_[j][0])
      {
        center_Of_TargetX = (corner_points_[i][0] + corner_points_[j][0]) / 2;
      }
      if(corner_points_[i][1] != corner_points_[j][1])
      {
        center_Of_TargetY = (corner_points_[i][1] + corner_points_[j][1]) / 2;
      }
    }
  }
  center_point[0]=center_Of_TargetX;
  center_point[1]=center_Of_TargetY;
  return center_point;
}

geometry_msgs::PoseArray check_if_point_in_pic::create_all_poses(double poseHeight, double spacing_in_z, double angleOfCone, int numberOfStopsForPhotos, Eigen::Vector2d center_point_of_target )
{
  double pi = 3.14159265359;
  double angleInRadians = angleOfCone*(pi/180);
  ROS_INFO ("angle= %lf degrees", angleInRadians *pi/180.0 );
  geometry_msgs::PoseArray msg;
  int extra_Counter = 0;
  for(double j=0; j<=poseHeight; j=j+spacing_in_z)
  {
    for(double l = (std::tan(angleInRadians))*spacing_in_z ; l<=(std::tan(angleInRadians) * (j) ); l = l+(std::tan(angleInRadians))*spacing_in_z)
    {
      EigenSTL::vector_Affine3d cameraPoses = getConicalPoses(numberOfStopsForPhotos, j, l);
      msg.header.frame_id = "target";
      msg.header.stamp = ros::Time::now();
      extra_Counter ++;
      for(int i=0; i<cameraPoses.size(); i++ )
      {
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(cameraPoses[i], pose);
        ROS_INFO("adding a pose");
        pose.position.x = pose.position.x + center_point_of_target[0];

        pose.position.y = pose.position.y + center_point_of_target[1];

        msg.poses.push_back(pose);

      }
    }
    numberOfStopsForPhotos ++;
  }
  return msg;
}

geometry_msgs::PoseArray check_if_point_in_pic::pose_filters(geometry_msgs::PoseArray msg2, tf::StampedTransform tf_transform, int image_width, int image_height, EigenSTL::vector_Affine3d AllcameraPoses, double fx, double fy, double cx, double cy )
{
  CreateChain::chain_creation reachability_filter;
  int num_created_poses = AllcameraPoses.size();
  for(int j=0;j<num_created_poses;j++)
  {
      bool accept_pose = true;
      for(int i=0;i<4;i++)
      {
          double u=0;
          double v=0;
          imagePoint(corner_points_[i], fx, fy, cx_, cy_, u, v,AllcameraPoses[j]);
          if(u<0 || u>=image_width || v<0 || v>= image_height )
          {
            ROS_ERROR("u,v = %lf %lf but image size = %d %d",u,v,image_width,image_height);
            accept_pose = false;
          }
          double x_part = tf_transform.getOrigin().x();
          double y_part = tf_transform.getOrigin().y();
          double z_part = tf_transform.getOrigin().z();

          //ROS_INFO("%lf %lf %lf", tf_transform.getOrigin().x(), tf_transform.getOrigin().y(), tf_transform.getOrigin().z());

          tf::Quaternion Q;
          tf_transform.getBasis().getRotation(Q);
          Eigen::Affine3d R;
          R.setIdentity();
          R.translate(Eigen::Vector3d(x_part,y_part,z_part));
          R.rotate(Eigen::Quaterniond(Q.getW(), Q.getX(), Q.getY(), Q.getZ()));
          Eigen::Matrix3Xd m = R.rotation();
          Eigen::Vector3d vec = R.translation();
          if(!reachability_filter.chain_Parse(R*AllcameraPoses[j]))
          {
            ROS_ERROR("Robot unable to reach the location");
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
  return msg2;
}

void check_if_point_in_pic::create_transform_listener(tf::StampedTransform& tf_transform,tf::TransformListener& tf_listen)
{
  ros::Time now = ros::Time::now();
  while (!tf_listen.waitForTransform(from_frame_param_, to_frame_param_, now, ros::Duration(1.0)))
   {
     now = ros::Time::now();
     ROS_INFO("waiting for tranform from %s to %s", from_frame_param_.c_str(), to_frame_param_.c_str());
   }
   try
   {
    tf_listen.lookupTransform(from_frame_param_, to_frame_param_, now, tf_transform);
   }
   catch (tf::TransformException& ex)
   {
     ROS_ERROR("%s", ex.what());
   }
}
check_if_point_in_pic::check_if_point_in_pic(ros::NodeHandle pivnh)
{
  //gets camera parameters
  if (!pivnh.getParam("image_width", image_width)){
    ROS_ERROR("did not set parameter image_width");
  }
  if (!pivnh.getParam("image_height", image_height)){
    ROS_ERROR("did not set parameter image_height");
  }
  if (!pivnh.getParam("focal_length", fx)){
    ROS_ERROR("did not set parameter focal_length");
  }
  if (!pivnh.getParam("numberOfStopsForPhotos", numberOfStopsForPhotos)){
    ROS_ERROR("did not set parameter numberOfStopsForPhotos");
  }
  if (!pivnh.getParam("poseHeight", poseHeight)){
    ROS_ERROR("did not set parameter poseHeight");
  }
  if (!pivnh.getParam("spacing_in_z", spacing_in_z)){
    ROS_ERROR("did not set parameter spacing_in_z");
  }
  if (!pivnh.getParam("angleOfCone", angleOfCone)){
    ROS_ERROR("did not set parameter angleOfCone");
  }

  fy = fx;
  cx_ = image_width/2;//assumes that lense is ligned up with sensor
  cy_ = image_height/2;//assums that lense is ligned up with sensor

  if (!pivnh.getParam("world", from_frame_param_))
  {
    ROS_ERROR("did not set parameter world");
  }
  if (!pivnh.getParam("target", to_frame_param_))
  {
    ROS_ERROR("did not set parameter target");
  }

  create_transform_listener(tf_transform, tf_listener);

  //reads in information about target
  string targetFile = "/home/lawrencelewis/catkin_ws/src/industrial_calibration/industrial_extrinsic_cal/yaml/ical_srv_target.yaml";
  vector<boost::shared_ptr<Target>>  myYamlDefinedTargets;
  if(!parseTargets(targetFile , myYamlDefinedTargets))
  {
    ROS_ERROR("could not parse target file %s", targetFile.c_str());
    return ;
  }
   boost::shared_ptr<Target> Mytarget = myYamlDefinedTargets[0];

  //finds the max values to be used as corner points
  ROS_INFO("target name = %s num_points = %d size of pts_ = %d",Mytarget->target_name_,Mytarget->num_points_,(int)Mytarget->pts_.size());
  xMax = Mytarget->pts_[0].x;
  xMin = Mytarget->pts_[0].x;
  yMax = Mytarget->pts_[0].y;
  yMin = Mytarget->pts_[0].y;
  for(int i=0;i<Mytarget->num_points_;i++)
  {
    if(Mytarget->pts_[i].x > xMax) xMax = Mytarget->pts_[i].x;
    if(Mytarget->pts_[i].y > yMax) yMax = Mytarget->pts_[i].y;
    if(Mytarget->pts_[i].x < xMin) xMin = Mytarget->pts_[i].x;
    if(Mytarget->pts_[i].y < yMin) yMin = Mytarget->pts_[i].y;
  }

  corner_points_[0] = Eigen::Vector3d((xMin-xMax/2)/5,(yMin-yMax/2)/5,0);
  corner_points_[1] = Eigen::Vector3d((xMax-xMax/2)/5,(yMin-yMax/2)/5,0);
  corner_points_[2] = Eigen::Vector3d((xMin-xMax/2)/5,(yMax-yMax/2)/5,0);
  corner_points_[3] = Eigen::Vector3d((xMax-xMax/2)/5,(yMax-yMax/2)/5,0);
}
void check_if_point_in_pic::create_rviz_target( geometry_msgs::PoseArray& msg)
{
  geometry_msgs::Pose pnt1,pnt2,pnt3,pnt4;
  pnt1.position.x = corner_points_[0][0];
  pnt1.position.y = corner_points_[0][1];
  pnt1.position.z = corner_points_[0][2];
  pnt1.orientation.w = 1.0;
  pnt1.orientation.x = 0.0;
  pnt1.orientation.y = 0.0;
  pnt1.orientation.z = 0.0;

  pnt2.position.x = corner_points_[1][0];
  pnt2.position.y = corner_points_[1][1];
  pnt2.position.z = corner_points_[1][2];
  pnt2.orientation.w = 1.0;
  pnt2.orientation.x = 0.0;
  pnt2.orientation.y = 0.0;
  pnt2.orientation.z = 0.0;

  pnt3.position.x = corner_points_[2][0];
  pnt3.position.y = corner_points_[2][1];
  pnt3.position.z = corner_points_[2][2];
  pnt3.orientation.w = 1.0;
  pnt3.orientation.x = 0.0;
  pnt3.orientation.y = 0.0;
  pnt3.orientation.z = 0.0;

  pnt4.position.x = corner_points_[3][0];
  pnt4.position.y = corner_points_[3][1];
  pnt4.position.z = corner_points_[3][2];
  pnt4.orientation.w = 1.0;
  pnt4.orientation.x = 0.0;
  pnt4.orientation.y = 0.0;
  pnt4.orientation.z = 0.0;

  msg.poses.push_back(pnt1);
  msg.poses.push_back(pnt2);
  msg.poses.push_back(pnt3);
  msg.poses.push_back(pnt4);

  return;
}
