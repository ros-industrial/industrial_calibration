//This program checks to see if the camera can see the target at different poses.
//Assumes a flat rectangular target.

#include <iostream>
#include <vector>
#include <industrial_extrinsic_cal/conical_pose_generator.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <industrial_extrinsic_cal/target.h>


using std::vector;

// local function prototypes
void imagePoint(const vector<double> &TargetPoint, double fx, double fy, double cx, double cy, double &u, double &v,Eigen::Affine3d cameraTransform);

void some_funciton_name(){
  ros::NodeHandle n;
  double Tx, Ty, Tz;
  //TODO take in cordinates of dots, focal lengths, camera resolution, camera principle points

  int image_width;
  int image_height;
  double fy, fx, cx, cy;
  int numberOfStopsForPhotos;
  double poseRadius;
  double poseHeight;

  // supposed to get the the cordinates of the dots
  vector<boost::shared_ptr<target> myYamlDefinedTargets;
  bool parseTargets(checkerboard8x8_target_def.yaml,myYamlDefinedTargets);

  Target* Mytarget = &myYamlDefinedTargets[0];

  double xMax,yMax,xMin,yMin;
  xMax = Mytarget->pts_[0].x;
  xMin = Mytarget->pts_[0].x;
  yMax = Mytarget->pts_[0].y;
  yMin = Mytarget->pts_[0].y;
  for(int i=0;i<Mytarget->num_pts_;i++){
    if(Mytarget->pts[i].x > xMax) xMax = Mytarget->pts_[i].x;
    if(Mytarget->pts[i].y > yMax) yMax = Mytarget->pts_[i].y;
    if(Mytarget->pts[i].x < xMin) xMin = Mytarget->pts_[i].x;
    if(Mytarget->pts[i].y < yMin) xMin = Mytarget->pts_[i].y;
  }
  Eigen::Vector3d corner_points[4];
  corner_points[0] = Eigen:Vector3d(xMin,yMin);
  corner_points[1] = Eigen:Vector3d(xMax,yMin);
  corner_points[2] = Eigen:Vector3d(xMin,yMax);
  corner_points[3] = Eigen:Vector3d(xMax,yMax);

  // creates poses. makes numberOfStopsForPhotos*3+1 different poses stored in "frames
  getConicalPoses(numberOfStopsForPhotos, poseHeight, poseRadius);

  vector<Eigen:Transform> accepted_poses;
  for(int j=0;j<num_poses,j++){
    bool accept_pose = true;
    for(int i=0;i<4;i++){
      double u,v;
      imagePoint(corner_point[i], fx, fy, cx, cy, u, v,cameraTransform[j]);
      if(u<0 || u>=image_width || v<0 || v>= image_height ) {
        accept_pose = false;
      }
    }
    if(accept_pose){
      accepted_poses.push_back(camera_transform[j]);
    }
  }

}


// Transforms a point into the camera frame then projects the new point into the 2d screen
// TODO fix syntax for imagePoint
void imagePoint(const vector<double> &TargetPoint, double fx, double fy, double cx, double cy, double &u, double &v,Eigen::Affine3d cameraTransform)
{
 double xyzTargetPoint= {TargetPoint[0],TargetPoint[1],TargetPoint[2]};
 pointInCameraFrame = cameraTransform.inverse() * xyzTargetPoint;
 double px = pointInCameraFrame[0];
 double py = pointInCameraFrame[1];
 double pz = pointInCameraFrame[2];
 u = fx*px/pz + cx;
 v = fy*py/pz + cy;
}

