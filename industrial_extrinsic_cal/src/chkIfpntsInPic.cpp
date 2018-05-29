/*This program checks to see if a point on the target
 *target is inside the camera's field of vision
 */


//function that transforms the point on the target into the cordinates of the camera
//and projects that new cordinate into the camera's  image

#include <iostream>
#include <vector>
#include "conical_pose_generator.h"

using namespace std;
double Tx; Ty; Tz;
double pointToCheckInTargetFrame[3] = { Tx, Ty, Tz, 1};

std::vector<double> pointInCamera;

pointInCamera = pointToCheck*frames[i];





// vector that will hold point x value in first space and y value in second space
vector<double> imagePoint;

std::vector<double> imagePoint(vector<double> pointInCamera, double focalX,double xoverz,double focalY, double centerOfImgX, double centerOfImgY){







return {u,v}
}

 
