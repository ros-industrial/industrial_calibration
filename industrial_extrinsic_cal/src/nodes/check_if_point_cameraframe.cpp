#include <ros/ros.h>
#include <industrial_extrinsic_cal/conical_pose_generator.h>
#include <tar.h>

bool callback(msgreqesttype input,
              msgresponsetype output)
{
  // DO logic here and fill in the message response

  // Take in a list of points (4 points that define the target's location)
  number_of_points = 4;
  double corners_of_target [number_of_points][3];

  // Find the min x, max x, min y , max y
  // TODO figure out how to find length of array and recieve the target
  target rectangleGrid;
  double xMin = rectangleGrid[0]; yMin=rectangleGrid[0]; xMax = rectangleGrid[0]; yMax = rectangleGrid[0];
  for(int q = 0; q < sizeof(targeta.pts); q++ )
  {
    if(target.pts_[q].x <= xMin)
    {
      xMin = target.pts_[q].x;
    }
    if(target.pts_[q].y <= yMin)
    {
      xMin = target.pts_[q].y;
    }
    if(target.pts_[q].x >= xMax)
    {
      xMax = target.pts_[q].x;
    }
    if(target.pts_[q].y >= yMax)
    {
      yMax = target.pts_[q].y;
    }
  }

  double botLetCorner[3] = {xMin, yMin, 0};
  double botRiteCorner[3] = {xMax, yMin, 0};
  double topRiteCorner[3] = {xMax, yMax, 0};
  double topLetCorner[3] = {xMin, yMax, 0};

  int maxPixlex;
  int maxPixley;


  //TODO Create an array that holds all points or a vector that hods all points we want to check

  double pointHolder[numberofpoints];

  // Creates all locations of the camera
  int number_of_poses;
  double height_of_cone;
  double radius_of_cone;

  //TODO Create an array/vector that holds all of the points in the camera frame

  double pointsInCameraThreeD[(number_of_poses*3+1)][number_of_points];
  double pointsInCameraTwoD[(number_of_poses*3+1)][number_of_points];
  // TODO create the inverted transform

  invertedTransforms(j);

  // will create "number_of_poses" * 3 + 1 number of poses
  getConicalPoses(number_of_poses,height_of_cone,radius_of_cone);

  for(int j = 0; j<(number_of_poses*3+1); j++)
  {
    // Finds location of Each point in the camera frame
    for(int i = 0; i < number_of_points; i++)
    {
      //TODO create the inverted transform
      // transfore from target to camera prespective
      pointsInCameraThreeD[j][i] = invertedTransforms(j) * pointHolder[i];
      // project camera perspective into 2D pixle perspective
      pointsInCameraTwoD[j][i] = pointsInCameraThreeD[j][i]*

    }
  }
  // Checks if the points are inside the camera frame

  // Create a dot product calculator


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_if_point_cameraframe");
  ros::NodeHandle nh;

  // create a ros server object

  // register the callback function with the server

  ros::spin();

  return 0;
}
