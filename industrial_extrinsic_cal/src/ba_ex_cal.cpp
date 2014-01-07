#include <stdlib.h>
#include <ostream>
#include <stdio.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>
#include <ros/ros.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <boost/foreach.hpp>

using std::string;
using std::vector;
using std::rand;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::P_BLOCK;


typedef struct
{
  double x; // image x
  double y; // image y
} Observation;


typedef struct
{
  union
  {
    struct
    {
      double PB_extrinsics[6]; // parameter block for intrinsics
      double PB_intrinsics[4]; // parameter block for extrinsics
    };
    struct
    {
      double angle_axis[3];	// angle axis data
      double position[3];	// position data
      double focal_length_x;	// focal length x
      double focal_length_y;	// focal length y
      double center_x;		// center x
      double center_y;		// center y
    };
  };
  string camera_name;
  int height;
  int width;
} Camera;

class ObservationDataPoint{
public:
  ObservationDataPoint(Camera* camera, int point_id, P_BLOCK p_position, Observation obs)
  {
    camera_ = camera;
    point_id_ = point_id;
    point_position_ = p_position;
    image_loc_ = obs;
  }
  ;

  ~ObservationDataPoint()
  {
  }
  ;

  Camera* camera_;
  int point_id_;
  P_BLOCK camera_extrinsics_;
  P_BLOCK point_position_;
  Observation image_loc_;
};
// end of class ObservationDataPoint


// local prototypes
void print_QTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);
void print_AATasH(double x, double y, double z, double tx, double ty, double tz);
void print_AATasHI(double x, double y, double z, double tx, double ty, double tz);
void print_AAasEuler(double x, double y, double z);
void print_camera(Camera C, string words);
void AAT2HI(double ax, double ay, double ay, double tx, double ty, double tz, HI[3][3])
{
  double R[9];
  double aa[3];
  aa[0] = ax;
  aa[1] = ay;
  aa[2] = az;
  ceres::AngleAxisToRotationMatrix(aa, R);
  HI[0][0]  = R[0];   HI[0][1] = R[1];  HI[0][2] = R[2];
  HI[1][0]  = R[3];   HI[1][1] = R[4];  HI[1][2] = R[5];
  HI[2][0]  = R[6];   HI[2][1] = R[7];  HI[2][2] = R[8];
  HI[3][0]  = 0.0;    HI[3][1] =  0.0;  HI[3][2] = 1.0;

  HI[0][3]  = -(tx * R[0] + ty * R[1] + tz * R[2]);
  HI[1][3]  = -(tx * R[3] + ty * R[4] + tz * R[5]);
  HI[2][3]  = -(tx * R[6] + ty * R[7] + tz * R[8]);
}

Observation project_point_no_distortion(Camera C, Point3d P);

// computes image of point in cameras image plane
Observation project_point_no_distortion(Camera C, Point3d P)
{
  double p[3];
  double pt[3];
  pt[0] = P.x;
  pt[1] = P.y;
  pt[2] = P.z;

  // transform point into camera frame
  // note, camera transform takes points from camera frame into world frame

  ceres::AngleAxisRotatePoint(C.angle_axis, pt, p);
  printf("point %6.3lf %6.3lf %6.3lf rotated: %6.3lf %6.3lf %6.3lf ", P.x, P.y, P.z, p[0], p[1], p[2]);
  p[0] += C.position[0];
  p[1] += C.position[1];
  p[2] += C.position[2];
  printf("translated: %6.3lf %6.3lf %6.3lf\n", p[0], p[1], p[2]);
  //  printf("PPP %6.3lf  %6.3lf %6.3lf \n",p[0],p[1],p[2]);
  double xp = p[0] / p[2];
  double yp = p[1] / p[2];

  // perform projection using focal length and camera center into image plane
  Observation O;
  O.x = C.focal_length_x * xp + C.center_x;
  O.y = C.focal_length_y * yp + C.center_y;
  return (O);
}

struct CameraReprjErrorNoDistortion
{
  CameraReprjErrorNoDistortion(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
    ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
  }

  template<typename T>
  bool operator()(const T* const c_p1, /** extrinsic parameters */
		  const T* point, /** point being projected, yes this is has 3 parameters */
		  T* resid) const
  {
    /** extract the variables from the camera parameters */
    int q = 0; /** extrinsic block of parameters */
    const T& x = c_p1[q++]; /**  angle_axis x for rotation of camera		 */
    const T& y = c_p1[q++]; /**  angle_axis y for rotation of camera */
    const T& z = c_p1[q++]; /**  angle_axis z for rotation of camera */
    const T& tx = c_p1[q++]; /**  translation of camera x */
    const T& ty = c_p1[q++]; /**  translation of camera y */
    const T& tz = c_p1[q++]; /**  translation of camera z */

    /** rotate and translate points into camera frame */
    T aa[3];/** angle axis  */
    T p[3]; /** point rotated */
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
    ceres::AngleAxisRotatePoint(aa, point, p);

    /** apply camera translation */
    T xp1 = p[0] + tx; /** point rotated and translated */
    T yp1 = p[1] + ty;
    T zp1 = p[2] + tz;

    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    /** perform projection using focal length and camera center into image plane */
    resid[0] = T(fx_) * xp + T(cx_) - T(ox_);
    resid[1] = T(fy_) * yp + T(cy_) - T(oy_);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, 
				     const double fx, const double fy,
				     const double cx, const double cy)
  {
    return (new ceres::AutoDiffCostFunction<CameraReprjErrorNoDistortion, 2, 6, 3>(new CameraReprjErrorNoDistortion(o_x, o_y, fx, fy, cx, cy)));
  }
  double ox_; /** observed x location of object in image */
  double oy_; /** observed y location of object in image */
  double fx_; /*!< known focal length of camera in x */
  double fy_; /*!< known focal length of camera in y */
  double cx_; /*!< known optical center of camera in x */
  double cy_; /*!< known optical center of camera in y */
};


int main(int argc, char** argv)
{
  vector<Point3d> points;
  vector<Camera>  cameras;
  vector<ObservationDataPoint> observations;

  google::InitGoogleLogging(argv[0]);
  if (argc != 3)
    {
      std::cerr << "usage: BaExCal <3Dpoints_file> <cameras_file>\n";
      return 1;
    }


  std::ifstream points_input_file(argv[1]);
  if (points_input_file.fail())
    {
      string temp(argv[1]);
      ROS_ERROR_STREAM("ERROR can't open points_input_file:  "<< temp.c_str());
      return (false);
    }

  std::ifstream cameras_input_file(argv[2]);
  if (cameras_input_file.fail())
    {
      string temp(argv[2]);
      ROS_ERROR_STREAM("ERROR can't open cameras_input_file:  "<< temp.c_str());
      return (false);
    }


  // parse points
  try
    {
      YAML::Parser points_parser(points_input_file);
      YAML::Node points_doc;
      points_parser.GetNextDocument(points_doc);

      // read in all points
      const YAML::Node *points_node = points_doc.FindValue("points");
      for (int j = 0; j < points_node->size(); j++){
	const YAML::Node *pnt_node = (*points_node)[j].FindValue("pnt");
	vector<float> temp_pnt;
	(*pnt_node) >> temp_pnt;
	Point3d temp_pnt3d;
	temp_pnt3d.x = temp_pnt[0];
	temp_pnt3d.y = temp_pnt[1];
	temp_pnt3d.z = temp_pnt[2];
	points.push_back(temp_pnt3d);
      }
    }
  catch (YAML::ParserException& e){
    ROS_INFO_STREAM("Failed to read points file ");
  }
  ROS_INFO_STREAM("Successfully read in " <<(int) points.size() << " points");


  Camera temp_camera;
  // parse cameras
  try
    {
      YAML::Parser camera_parser(cameras_input_file);
      YAML::Node camera_doc;
      camera_parser.GetNextDocument(camera_doc);

      // read in all static cameras
      if (const YAML::Node *camera_parameters = camera_doc.FindValue("cameras")){
	ROS_INFO_STREAM("Found "<<camera_parameters->size()<<" cameras ");
	for (unsigned int i = 0; i < camera_parameters->size(); i++){
	  (*camera_parameters)[i]["camera_name"]    >> temp_camera.camera_name;
	  // replaced with rotation matrix
	  //        (*camera_parameters)[i]["angle_axis_ax"]  >> temp_camera.angle_axis[0];
	  //        (*camera_parameters)[i]["angle_axis_ay"]  >> temp_camera.angle_axis[1];
	  //        (*camera_parameters)[i]["angle_axis_az"]  >> temp_camera.angle_axis[2];
	  const YAML::Node *rotation_node = (*camera_parameters)[i].FindValue("rotation");
	  if(rotation_node->size() != 9){
	    ROS_ERROR_STREAM("Rotation " << i << " should have 9 pts, has " << (int)rotation_node->size());
	  }
	  // read in the transform, transposed since camera parameters
          // project world points into camera frame, not camera to world
	  double R[9],tx,ty,tz;
	  (*rotation_node)[0]  >> R[0];
	  (*rotation_node)[1]  >> R[3];
	  (*rotation_node)[2]  >> R[6];
	  (*rotation_node)[3]  >> R[1];
	  (*rotation_node)[4]  >> R[4];
	  (*rotation_node)[5]  >> R[7];
	  (*rotation_node)[6]  >> R[2];
	  (*rotation_node)[7]  >> R[5];
	  (*rotation_node)[8]  >> R[8];
	  (*camera_parameters)[i]["position_x"]     >> tx;
	  (*camera_parameters)[i]["position_y"]     >> ty;
	  (*camera_parameters)[i]["position_z"]     >> tz;
	  temp_camera.position[0] = -(tx * R[0] + ty * R[3] + tz * R[6]);
	  temp_camera.position[1] = -(tx * R[1] + ty * R[4] + tz * R[7]);
	  temp_camera.position[2] = -(tx * R[2] + ty * R[5] + tz * R[8]);
	  ceres::RotationMatrixToAngleAxis(R,temp_camera.angle_axis);
	  (*camera_parameters)[i]["focal_length_x"] >> temp_camera.focal_length_x;
	  (*camera_parameters)[i]["focal_length_y"] >> temp_camera.focal_length_y;
	  (*camera_parameters)[i]["center_x"]       >> temp_camera.center_x;
	  (*camera_parameters)[i]["center_y"]       >> temp_camera.center_y;
	  (*camera_parameters)[i]["width"]          >> temp_camera.width;
	  (*camera_parameters)[i]["height"]         >> temp_camera.height;
	  cameras.push_back(temp_camera);
	}	// end of for each camera in file
      } // end if there are any cameras in file
    }
  catch (YAML::ParserException& e){
    ROS_INFO_STREAM("Failed to read in moving cameras from yaml file ");
    ROS_INFO_STREAM("camera name    = " << temp_camera.camera_name.c_str());
    ROS_INFO_STREAM("angle_axis_ax  = " << temp_camera.angle_axis[0]);
    ROS_INFO_STREAM("angle_axis_ay  = " << temp_camera.angle_axis[1]);
    ROS_INFO_STREAM("angle_axis_az  = " << temp_camera.angle_axis[2]);
    ROS_INFO_STREAM("position_x     = " << temp_camera.position[0]);
    ROS_INFO_STREAM("position_y     = " << temp_camera.position[1]);
    ROS_INFO_STREAM("position_z     = " << temp_camera.position[2]);
    ROS_INFO_STREAM("focal_length_x = " << temp_camera.focal_length_x);
    ROS_INFO_STREAM("focal_length_y = " << temp_camera.focal_length_y);
    ROS_INFO_STREAM("center_x       = " << temp_camera.center_x);
    ROS_INFO_STREAM("center_y       = " << temp_camera.center_y);
  }
  ROS_INFO_STREAM("Successfully read in %d cameras " << (int) cameras.size());


  // create nominal observations of points using camera parameters
  for(int i=0; i < (int)cameras.size(); ++i){
    for(int j=0; j < (int)points.size(); j++){
      // find image of point in camera
      Observation observation = project_point_no_distortion(cameras[i], points[j]);
      if(observation.x>=0 &&observation.x<cameras[i].width &&
	 observation.y>=0 &&observation.y<cameras[i].height ){
	// save observation
	ObservationDataPoint new_obs(&(cameras[i]),j,&(points[i].pb[0]),observation);
	observations.push_back(new_obs);
      }
    }
  }
  ROS_INFO_STREAM("found " << observations.size() << " observations");

  // 
  // Setup problem 1
  // perturb camera positions and orientations
  // submit nominal observations as cost functions

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem1;
  BOOST_FOREACH(ObservationDataPoint obs, observations){
    double x  = obs.image_loc_.x;
    double y  = obs.image_loc_.y;
    double fx = obs.camera_->focal_length_x;
    double fy = obs.camera_->focal_length_y;
    double cx = obs.camera_->center_x;
    double cy = obs.camera_->center_y;

    ceres::CostFunction* cost_function = CameraReprjErrorNoDistortion::Create(x,y,fx,fy,cx,cy);
      
    double *extrinsics = obs.camera_->PB_extrinsics;
    double *points     = obs.point_position_;
    problem1.AddResidualBlock(cost_function, NULL, extrinsics, points);

    // set extrinsic constant location, because something needs to be set
    static bool first_camera = true;
    if(first_camera){
      first_camera = false;
      problem1.SetParameterBlockConstant(extrinsics);
    }
  }
  BOOST_FOREACH(Camera C, cameras){
    static bool first_camera = true;
    if(!first_camera){		// perturb location by as much as 12 inches in any direction
      C.position[0] += ((double)(std::rand()%24)-12)/12.0;
      C.position[1] += ((double)(std::rand()%24)-12)/12.0;
      C.position[2] += ((double)(std::rand()%24)-12)/12.0;
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem1, &summary);
  std::cout << summary.FullReport() << "\n";
  for(int i=0;i<(int)cameras.size();i++){
    char temp[100];
    sprintf(temp,"Camera %s final parameters",cameras[i].camera_name.c_str());
    print_camera(cameras[i], temp);
  }
  // Setup problem 2
  // perturb point positions 
  // submit nominal camera extrinsics
  // solve and compare to original point values

  // Setup problem 3
  // use original point positions, set constant
  // use original camera poses
  // for a statistically significant number of iterations:
  //     perturb observations
  //     solve and compare to original pose values
  // end for
  // calculate statistics on poses as an estimate of pose quality

  return 0;
}

// print a quaternion plus position as a homogeneous transform
void print_QTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz)
{
  double Rs11 = qw * qw + qx * qx - qy * qy - qz * qz;
  double Rs21 = 2.0 * qx * qy + 2.0 * qw * qz;
  double Rs31 = 2.0 * qx * qz - 2.0 * qw * qy;

  double Rs12 = 2.0 * qx * qy - 2.0 * qw * qz;
  double Rs22 = qw * qw - qx * qx + qy * qy - qz * qz;
  double Rs32 = 2.0 * qy * qz + 2.0 * qw * qx;

  double Rs13 = 2.0 * qx * qz + 2.0 * qw * qy;
  double Rs23 = 2.0 * qy * qz - 2.0 * qw * qx;
  double Rs33 = qw * qw - qx * qx - qy * qy + qz * qz;

  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs11, Rs12, Rs13, tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs21, Rs22, Rs23, ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs31, Rs32, Rs33, tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

// angle axis to homogeneous transform inverted
void print_AATasH(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[3], R[6], tx);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[1], R[4], R[7], ty);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[2], R[5], R[8], tz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

// angle axis to homogeneous transform
void print_AATasHI(double x, double y, double z, double tx, double ty, double tz)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  double ix = -(tx * R[0] + ty * R[1] + tz * R[2]);
  double iy = -(tx * R[3] + ty * R[4] + tz * R[5]);
  double iz = -(tx * R[6] + ty * R[7] + tz * R[8]);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[1], R[2], ix);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[3], R[4], R[5], iy);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[6], R[7], R[8], iz);
  printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
}

void print_AAasEuler(double x, double y, double z)
{
  double R[9];
  double aa[3];
  aa[0] = x;
  aa[1] = y;
  aa[2] = z;
  ceres::AngleAxisToRotationMatrix(aa, R);
  double rx = atan2(R[7], R[8]);
  double ry = atan2(-R[6], sqrt(R[7] * R[7] + R[8] * R[8]));
  double rz = atan2(R[3], R[0]);
  printf("rpy = %8.4f %8.4f %8.4f\n", rx, ry, rz);
}
void print_camera(Camera C, string words)
{
  printf("%s\n", words.c_str());
  printf("Camera to World Transform:\n");
  print_AATasHI(C.angle_axis[0], C.angle_axis[1], C.angle_axis[2], 
		C.position[0],   C.position[1],   C.position[2]);

  printf("World to Camera\n");
  print_AATasH(C.angle_axis[0], C.angle_axis[1], C.angle_axis[2], 
	       C.position[0],   C.position[1],   C.position[2]);

  print_AAasEuler(C.angle_axis[0], C.angle_axis[1], C.angle_axis[2]);
  printf("fx = %8.3lf fy = %8.3lf\n", C.focal_length_x, C.focal_length_y);
  printf("cx = %8.3lf cy = %8.3lf\n", C.center_x, C.center_y);
}
