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
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/camera_yaml_parser.h>
#include <industrial_extrinsic_cal/targets_yaml_parser.h>
#include <industrial_extrinsic_cal/points_yaml_parser.h>
#include <industrial_extrinsic_cal/observation_data_point.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#define SHOW_DEBUG false

using std::ifstream;
using std::string;
using std::vector;
using boost::shared_ptr;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::P_BLOCK;
using industrial_extrinsic_cal::Camera;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::ObservationDataPoint;
using industrial_extrinsic_cal::extractCameraIntrinsics;
using industrial_extrinsic_cal::transformPoint;
using industrial_extrinsic_cal::projectPntNoDistortion;
using industrial_extrinsic_cal::string2CostType;
using industrial_extrinsic_cal::Cost_function;

typedef boost::minstd_rand base_gen_type;
base_gen_type gen(42);
boost::normal_distribution<> normal_dist(0, 1);  // zero mean unit variance
boost::variate_generator<base_gen_type&, boost::normal_distribution<> > randn(gen, normal_dist);

typedef struct observation
{
  double x;
  double y;
} Observation;

typedef struct scene
{
  int scene_id;
  Pose6d target_pose;
  std::vector<std::string> camera_names;
} Scene;

/*! Brief defines a camera with extra structures to maintain statistics */
class CameraWithHistory : public Camera
{
public:
  int height;
  int width;
  vector<Pose6d> pose_history;
  double pose_position_mean_error;
  double pose_position_sigma;
  double pose_orientation_mean_error;
  double pose_orientation_sigma;
  double sigma_ax, sigma_ay, sigma_az;
  int num_observations;
};

/*! Brief defines a 3d point with extra structures to maintain statistics*/
class Point3dWithHistory
{
public:
  Point3d point;
  vector<double> x_history;
  vector<double> y_history;
  vector<double> z_history;
  double mean_x;
  double mean_y;
  double mean_z;
  double sigma_x;
  double sigma_y;
  double sigma_z;
  int num_observations;
};

// local prototypes
void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);
void printAATasH(double x, double y, double z, double tx, double ty, double tz);
void printAATasHI(double x, double y, double z, double tx, double ty, double tz);
void printAAasEuler(double x, double y, double z);
void printCamera(CameraWithHistory C, string words);
void computeObservations(vector<CameraWithHistory>& cameras, vector<Point3dWithHistory>& points, double noise,
                         double max_dist, vector<ObservationDataPoint>& observations);
void perturbCameras(vector<CameraWithHistory>& cameras, double position_noise, double degrees_noise);
void perturbPoints(vector<Point3dWithHistory>& points, double position_noise);
void perturbPose(Pose6d& pose, double position_noise, double degree_noise);
void independentlyPerturbCameras(vector<CameraWithHistory>& cameras);
void copyCamerasWoHistory(vector<CameraWithHistory>& original_cameras, vector<CameraWithHistory>& cameras);
void copyPoints(vector<Point3dWithHistory>& original_points, vector<Point3dWithHistory>& points);
void addPoseToHistory(vector<CameraWithHistory>& cameras, vector<CameraWithHistory>& original_cameras);
void computeHistoricPoseStatistics(vector<CameraWithHistory>& cameras);
void addPointsToHistory(vector<Point3dWithHistory>& points, vector<Point3dWithHistory>& original_points);
void computeHistoricPointStatistics(vector<Point3dWithHistory>& points);
void compareCameras(vector<CameraWithHistory>& C1, vector<CameraWithHistory>& C2);
void compareObservations(vector<ObservationDataPoint>& O1, vector<ObservationDataPoint>& O2);
bool parseScenes(std::string& scene_file_name, std::vector<Scene>& scenes);
void showScenes(vector<Scene>& scenes);
void addTargetPoints(int rows, int cols, double spacing, vector<Point3dWithHistory>& points, Pose6d& target_pose);
void computeObservationsFromScenes(vector<Scene>& scenes,               // pose of target, and which cameras observed it
                                   vector<CameraWithHistory>& cameras,  // list of cameras
                                   double target_pos_noise,  // perturb position of target by this amount then compute
                                   double target_degree_noise,  // peturb orientation of target by this amount then
                                                                // compute
                                   double image_noise,          // amount of noise to add to each observation
                                   vector<ObservationDataPoint>& observations,  // returned data
                                   vector<Point3dWithHistory>& points);         // returned target points
ObservationDataPoint predictObservationOfPoint(CameraWithHistory& C, Point3dWithHistory& P, Pose6d& target_pose,
                                               int scene_id, int point_id, double image_noise,
                                               bool& results_within_image);
void computeObservationsOfPoints(vector<CameraWithHistory>& cameras, vector<Point3dWithHistory>& points,
                                 Pose6d& target_pose, int scene_id, vector<ObservationDataPoint>& observations,
                                 double camera_position_noise = 0.0, double camera_degree_noise = 0.0,
                                 double image_noise = 0.0);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nist_analysis");

  vector<Point3d> original_points_nh;               // original as read in, no history
  vector<Point3dWithHistory> original_points;       // working copy of original points
  vector<shared_ptr<Camera> > original_cameras_sp;  // cameras with given poses
  vector<CameraWithHistory> original_cameras;       // cameras with given poses

  vector<Point3dWithHistory> points;                 // working copy of original points
  vector<Point3d> original_field_points_nh;          // test points for accuracy estimation
  vector<Point3dWithHistory> original_field_points;  // test points for accuracy estimation
  vector<Point3dWithHistory> field_points;           // working copy of original field points

  vector<CameraWithHistory> cameras;                         // working copy of original cameras
  vector<ObservationDataPoint> original_observations;        // noisless observations of original points
  vector<ObservationDataPoint> original_field_observations;  // noisless observations field points
  vector<ObservationDataPoint> observations;                 // working observations

  // TODO use a parameter file for these, or make them arguments
  // hard coded constants
  double camera_pos_noise = 0.10;   // 25 cm
  double camera_or_noise = 3.0;     // degrees
  double point_pos_noise = 0.1;     // 10cm perturbation prior to triangulation
  double target_pos_noise = 0.003;  // 3mm
  double target_or_noise = 1.0;     // degrees
  double image_noise = 0.1;         // pixels
  int num_test_cases = 100;         // each test case is a statistical sample

  google::InitGoogleLogging(argv[0]);
  if (argc != 4)
  {
    std::cerr << "usage: NistAnalysis <scene_file> <cameras_file> <fieldpoints_file\n";
    return 1;
  }

  std::string scene_file_name;
  ifstream scene_file(argv[1]);
  scene_file_name = argv[1];
  if (scene_file.fail())
  {
    ROS_ERROR_STREAM("ERROR can't open scene_file:  " << scene_file_name.c_str());
    return (false);
  }

  std::string cameras_file(argv[2]);
  std::string field_points_file(argv[3]);

  // read in the camera data, observation data and field points from yaml files
  parseCameras(cameras_file, original_cameras_sp);

  // copy each into a history capable version
  // NOTE: pullTransform() is called which installs the results from extrinsic calibration in the camera data
  std::string ref_frame("world");
  BOOST_FOREACH (shared_ptr<Camera>& current_camera, original_cameras_sp)
  {
    CameraWithHistory temp_camera;
    temp_camera.setTransformInterface(current_camera->getTransformInterface());
    temp_camera.camera_observer_ = current_camera->camera_observer_;
    temp_camera.trigger_ = current_camera->trigger_;
    temp_camera.camera_parameters_ = current_camera->camera_parameters_;
    temp_camera.camera_name_ = current_camera->camera_name_;
    temp_camera.intermediate_frame_ = current_camera->intermediate_frame_;
    temp_camera.is_moving_ = current_camera->is_moving_;
    temp_camera.height = current_camera->camera_parameters_.height;
    temp_camera.width = current_camera->camera_parameters_.width;
    temp_camera.setTIReferenceFrame(ref_frame);
    temp_camera.pullTransform();
    original_cameras.push_back(temp_camera);
  }

  // parse the scenes.
  // This is the output from the extrinsic calibration script
  // every scene has an id, a location of the target, and a list of cameras that observed the target
  std::vector<Scene> scenes;
  parseScenes(scene_file_name, scenes);
  if (SHOW_DEBUG) showScenes(scenes);

  // create original_points and original_observations from scenes
  // Note, these are nominal, no observation data is kept from calibration
  // it is expected that the nominal is good enough for analyis of the gometry
  computeObservationsFromScenes(scenes, original_cameras, 0.0, 0.0, 0.0, original_observations, original_points);

  // parse the field points used to generate the accuracy map
  parsePoints(field_points_file, original_field_points_nh);
  original_field_points.clear();
  for (int i = 0; i < original_field_points_nh.size(); i++)
  {
    Point3dWithHistory temp_point;
    temp_point.point.x = original_field_points_nh[i].x;
    temp_point.point.y = original_field_points_nh[i].y;
    temp_point.point.z = original_field_points_nh[i].z;
    original_field_points.push_back(temp_point);
  }

  // Setup problem 1: A test to see if camera extrinsics may be
  // recovered with fixed fiducials.  Create nominal observations of
  // points using camera parameters. Perturb camera positions and
  // orientations. Then, add all observation cost functions to problem
  // and solve.
  copyCamerasWoHistory(original_cameras, cameras);
  copyPoints(original_points, points);
  // compute noise free observations
  // these should be exactly the same as the original observations and original points
  computeObservationsFromScenes(scenes, cameras, 0.0, 0.0, 0.0, observations, points);

  // compute noisy observations
  computeObservationsFromScenes(scenes, cameras, 0.0, 0.0, image_noise, observations, points);

  // compare noisy observations to noise free observations
  ROS_INFO("Comparing %d noiseless observations to %d noisy observations", (int)original_observations.size(),
           (int)observations.size());
  compareObservations(original_observations, observations);  // shows noise is correct

  // regenerate noiseless observations with unperturbed cameras, note observations point to cameras parameters
  computeObservationsFromScenes(scenes, cameras, 0.0, 0.0, 0.0, observations, points);

  // perturb cameras
  ROS_INFO("perturbing cameras");
  perturbCameras(cameras, camera_pos_noise, camera_or_noise);
  // compare the original camera's locations to the perturbed ones
  ROS_INFO("comparing perturbed cameras to original cameras");
  compareCameras(original_cameras, cameras);  // shows how far apart they started

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem1;
  // each observation is noisy and points to cameras whose extrinsics are perturbed
  // each point is a fiducial in a known/surveyed location
  ROS_INFO("solving with %d observations", (int)observations.size());
  int b1o = 0, b2o = 0, b3o = 0, b4o = 0, b5o = 0, b6o = 0, b7o = 0;
  BOOST_FOREACH (ObservationDataPoint obs, observations)
  {
    double* extrinsics = obs.camera_extrinsics_;
    Point3d point;
    point.x = obs.point_position_[0];
    point.y = obs.point_position_[1];
    point.z = obs.point_position_[2];
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    industrial_extrinsic_cal::extractCameraIntrinsics(obs.camera_intrinsics_, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    ceres::CostFunction* cost_function =
        industrial_extrinsic_cal::CameraReprjErrorPK::Create(obs.image_x_, obs.image_y_, fx, fy, cx, cy, point);
    problem1.AddResidualBlock(cost_function, NULL, extrinsics);
  }
  BOOST_FOREACH (CameraWithHistory& C, cameras)
  {
    ROS_INFO("%s has %d observations", C.camera_name_.c_str(), C.num_observations);
  }

  // solve problem
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1000;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem1, &summary);
  if (SHOW_DEBUG)
  {  // display results
    std::cout << summary.FullReport() << "\n";
  }

  // see how well original camera poses were recoved with noisy observations
  ROS_INFO("comparing camera poses derived from noiseless measurements to actual");
  compareCameras(original_cameras, cameras);  // shows that cameras were recovered

  // compute observations of cameras at solved poses to noiseless observations
  computeObservationsFromScenes(scenes, cameras, 0.0, 0.0, 0.0, observations, points);
  ROS_INFO("comparing original observations to those made from poses derived from noisy measurments");
  compareObservations(original_observations, observations);  // ceres minimized observation noise?

  ROS_ERROR("verify recovery of camera poses for cameras with observations");

  // now, add noise to observations and see how well camera poses are recoverd
  copyCamerasWoHistory(original_cameras, cameras);
  computeObservationsFromScenes(scenes, cameras, 0.0, 0.0, image_noise, observations, points);
  perturbCameras(cameras, camera_pos_noise, camera_or_noise);
  ceres::Problem problem2;
  BOOST_FOREACH (ObservationDataPoint obs, observations)
  {
    double* extrinsics = obs.camera_extrinsics_;
    Point3d point;
    point.x = obs.point_position_[0];
    point.y = obs.point_position_[1];
    point.z = obs.point_position_[2];
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    industrial_extrinsic_cal::extractCameraIntrinsics(obs.camera_intrinsics_, fx, fy, cx, cy, k1, k2, k3, p1, p2);
    ceres::CostFunction* cost_function =
        industrial_extrinsic_cal::CameraReprjErrorPK::Create(obs.image_x_, obs.image_y_, fx, fy, cx, cy, point);
    problem2.AddResidualBlock(cost_function, NULL, extrinsics);
  }
  // solve problem
  ceres::Solve(options, &problem2, &summary);
  if (SHOW_DEBUG)
  {  // display results
    std::cout << summary.FullReport() << "\n";
  }
  // see how well original camera poses were recoved with noisy observations
  ROS_INFO("comparing camera poses derived from noisy measurements to actual");
  compareCameras(original_cameras, cameras);  // shows that cameras were recovered

  ROS_ERROR("verify recovery of camera poses with reduced accuracy for cameras with observations");

  // Now we are ready to compute each camera's statistics
  // We answer the following questions
  // 1. what is the mean error in pose estimate for each camera
  // 2. what is the variance in pose estimate for each camera
  // These two values will be stored in camera.pos so that we can
  // use them to generate estimates of the accuracy of field point localization

  // compute poses for cameras for a bunch of test cases
  options.minimizer_progress_to_stdout = false;
  for (int test_case = 0; test_case < num_test_cases; test_case++)
  {
    copyCamerasWoHistory(original_cameras, cameras);
    computeObservationsFromScenes(scenes, cameras, 0.0, 0.0, image_noise, observations, points);
    perturbCameras(cameras, camera_pos_noise, camera_or_noise);

    ceres::Problem problem;
    BOOST_FOREACH (ObservationDataPoint obs, observations)
    {
      double x = obs.image_x_;
      double y = obs.image_y_;
      double* extrinsics = obs.camera_extrinsics_;
      Point3d point;
      point.x = obs.point_position_[0];
      point.y = obs.point_position_[1];
      point.z = obs.point_position_[2];
      double fx, fy, cx, cy, k1, k2, k3, p1, p2;
      industrial_extrinsic_cal::extractCameraIntrinsics(obs.camera_intrinsics_, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      ceres::CostFunction* cost_function =
          industrial_extrinsic_cal::CameraReprjErrorPK::Create(x, y, fx, fy, cx, cy, point);
      problem.AddResidualBlock(cost_function, NULL, extrinsics);
    }
    ceres::Solve(options, &problem, &summary);
    addPoseToHistory(cameras, original_cameras);
  }  // end of test cases
  computeHistoricPoseStatistics(original_cameras);

  BOOST_FOREACH (CameraWithHistory& C, original_cameras)
  {
    ROS_INFO("%s\t:mean_pos_error = %7.5lf sigma=  %7.5lf angular %7.5lf number_observations = %d",
             C.camera_name_.c_str(), C.pose_position_mean_error, C.pose_position_sigma,
             C.pose_orientation_sigma * 180 / 3.1415, C.num_observations);
  }

  // at this point cameras each have an uncertianty model.
  // the uncertianty model is the variance in both position and orientation
  // We can now sample from this model to get uncertianty estimates for field points.
  // for .25 pixel error, most cameras have between  in position variance
  // and have less than .9 degrees in angular variance

  // Now we are ready to compute the field accuracy of the working volume
  // To to this we do the following.
  // Create a grid of locations representative of the field accuracy
  // For this optimization, the points will be solve, while the cameras are fixed
  // The cameras will be fixed at their original/nominal locations
  // But the observations will be made from the perturbed/real locations with each test case
  // Loop:
  //    Perturb the cameras using each camera's pose accuracy estimate
  //    compute noisy observations from those perturbed cameras
  //    Reset cameras to nominal locations, and hold fixed (at incorrect locations)
  //    optimize to find locations of all field points
  // end Loop
  // Compute point statistics for each field point

  // Compute noiseless observations of field points
  // determine how many times each point gets observed (need at least 2 to be triangulated)
  copyCamerasWoHistory(original_cameras, cameras);
  Pose6d target_pose;  // not used, but needed to fill out the observation data structure
  int scene_id = 0;    // not used
  computeObservationsOfPoints(cameras, original_field_points, target_pose, scene_id, original_field_observations);

  independentlyPerturbCameras(cameras);
  ROS_ERROR("comparing original camera poses to those using independent perturbations\n");
  compareCameras(original_cameras, cameras);  // will have some agregate overall statistics

  for (int test_case = 0; test_case < num_test_cases; test_case++)
  {
    copyPoints(original_field_points, field_points);  // working copy of field points
    copyCamerasWoHistory(original_cameras, cameras);  // working copy of cameras

    // noiseless ideal observations of cameras at their actual locations, these observation now point toward field
    // points
    computeObservationsOfPoints(cameras, field_points, target_pose, scene_id, observations);
    // noisey ideal observations of cameras at their actual locations
    // computeObservationsOfPoints(cameras, field_points, target_pose, scene_id, observations, 0.0, 0.0, image_noise);

    // cameras independently perturbed from their actual locations for triangulation
    independentlyPerturbCameras(cameras);
    perturbPoints(field_points, point_pos_noise);

    ceres::Problem problem;
    options.minimizer_progress_to_stdout = false;
    BOOST_FOREACH (ObservationDataPoint obs, observations)
    {
      double x = obs.image_x_;
      double y = obs.image_y_;
      double* points = obs.point_position_;  // this is perturbed from ideal by point_pos_noise
      double fx, fy, cx, cy, k1, k2, k3, p1, p2;
      industrial_extrinsic_cal::extractCameraIntrinsics(obs.camera_intrinsics_, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      double tx, ty, tz, ax, ay, az;
      industrial_extrinsic_cal::extractCameraExtrinsics(obs.camera_extrinsics_, tx, ty, tz, ax, ay, az);
      Pose6d camera_pose(tx, ty, tz, ax, ay, az);  // note this is the pose from camera to world
      ceres::CostFunction* cost_function =
          industrial_extrinsic_cal::TriangulationError::Create(x, y, fx, fy, cx, cy, camera_pose);
      problem.AddResidualBlock(cost_function, NULL, points);
    }
    ceres::Solve(options, &problem, &summary);

    addPointsToHistory(field_points, original_field_points);
  }
  computeHistoricPointStatistics(original_field_points);
  FILE* fp = fopen("field_results.m", "w");
  fprintf(fp, "f = [\n");
  BOOST_FOREACH (Point3dWithHistory P, original_field_points)
  {
    printf("Mean Error= %9.5lf %9.5lf %9.5lf sigma= %9.5lf %9.5lf %9.5lf num_obs=%d\n", P.mean_x - P.point.x,
           P.mean_y - P.point.y, P.mean_z - P.point.z, P.sigma_x, P.sigma_y, P.sigma_z, P.num_observations);
    fprintf(fp, "%9.5lf %9.5lf %9.5lf %9.5lf %9.5lf %9.5lf %9.5lf %9.5lf %9.5lf;\n", P.point.x, P.point.y, P.point.z,
            P.mean_x, P.mean_y, P.mean_z, P.sigma_x, P.sigma_y, P.sigma_z);
  }
  fprintf(fp, "];\n");
  fclose(fp);
}

// print a quaternion plus position as a homogeneous transform
void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz)
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
void printAATasH(double x, double y, double z, double tx, double ty, double tz)
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
void printAATasHI(double x, double y, double z, double tx, double ty, double tz)
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

void printAAasEuler(double x, double y, double z)
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
void printCamera(CameraWithHistory C, string words)
{
  printf("%s\n", words.c_str());
  printf("Point in Camera frame to points in World frame Transform:\n");
  printAATasHI(C.camera_parameters_.angle_axis[0], C.camera_parameters_.angle_axis[1],
               C.camera_parameters_.angle_axis[2], C.camera_parameters_.position[0], C.camera_parameters_.position[1],
               C.camera_parameters_.position[2]);

  printf("Points in World frame to points in Camera frame transform\n");
  printAATasH(C.camera_parameters_.angle_axis[0], C.camera_parameters_.angle_axis[1],
              C.camera_parameters_.angle_axis[2], C.camera_parameters_.position[0], C.camera_parameters_.position[1],
              C.camera_parameters_.position[2]);

  printAAasEuler(C.camera_parameters_.angle_axis[0], C.camera_parameters_.angle_axis[1],
                 C.camera_parameters_.angle_axis[2]);
  printf("fx = %8.3lf fy = %8.3lf\n", C.camera_parameters_.focal_length_x, C.camera_parameters_.focal_length_y);
  printf("cx = %8.3lf cy = %8.3lf\n", C.camera_parameters_.center_x, C.camera_parameters_.center_y);
}

void computeObservations(vector<CameraWithHistory>& cameras, vector<Point3dWithHistory>& points, double noise,
                         double max_dist, vector<ObservationDataPoint>& observations)
{
  double pnoise = noise / sqrt(1.58085);  // magic number found by trial and error
  int n_observations = 0;
  observations.clear();

  // reset number of observations for the points
  BOOST_FOREACH (Point3dWithHistory& P, points)
  {
    P.num_observations = 0;
  }

  // create nominal observations of points using camera parameters
  BOOST_FOREACH (CameraWithHistory& C, cameras)
  {
    C.num_observations = 0;
    int j = 0;
    BOOST_FOREACH (Point3dWithHistory& P, points)
    {
      j++;
      // find image of point in camera
      double dx = C.camera_parameters_.position[0] - P.point.x;
      double dy = C.camera_parameters_.position[1] - P.point.y;
      double dz = C.camera_parameters_.position[2] - P.point.z;
      double distance = sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < max_dist)
      {
        double fx, fy, cx, cy, k1, k2, k3, p1, p2;
        Observation obs;
        double* intrinsics = C.camera_parameters_.pb_intrinsics;
        industrial_extrinsic_cal::extractCameraIntrinsics(intrinsics, fx, fy, cx, cy, k1, k2, k3, p1, p2);
        P_BLOCK point = &P.point.pb[0];
        industrial_extrinsic_cal::projectPntNoDistortion(point, fx, fy, cx, cy, obs.x, obs.y);
        obs.x += pnoise * randn();  // add observation noise
        obs.y += pnoise * randn();
        if (obs.x >= 0 && obs.x < C.width && obs.y >= 0 && obs.y < C.height)
        {
          // save observation
          Pose6d target_pose, intermediate_frame;
          industrial_extrinsic_cal::Cost_function cost_type;
          ObservationDataPoint new_obs(C.camera_name_,
                                       "who_cares",                         // target name
                                       2,                                   // target type
                                       0,                                   // scene id (who cares)
                                       C.camera_parameters_.pb_intrinsics,  // camera intrinsics
                                       C.camera_parameters_.pb_extrinsics,  // camera extrinsics
                                       0,                                   // point_id
                                       target_pose.pb_pose,                 // target_pose
                                       point,                               // point position
                                       obs.x,                               // observation x
                                       obs.x,                               // observation y
                                       cost_type,                           // const Cost_function
                                       intermediate_frame);                 // Pose6d of intermediate frame
          observations.push_back(new_obs);
          C.num_observations++;
          P.num_observations++;
          n_observations++;
        }  // end if observation within field of view
      }    // end if point close enough to camera
    }      // end for each fiducial point
  }        // end for each camera
}  // end compute observations

void perturbCameras(vector<CameraWithHistory>& cameras, double position_noise, double degrees_noise)
{
  double pos_noise = position_noise / sqrt(2.274625);
  double radian_noise = degrees_noise * 3.1415 / (180.0 * sqrt(2.58047));
  BOOST_FOREACH (CameraWithHistory& C, cameras)
  {
    C.camera_parameters_.position[0] += pos_noise * randn();
    C.camera_parameters_.position[1] += pos_noise * randn();
    C.camera_parameters_.position[2] += pos_noise * randn();
    C.camera_parameters_.angle_axis[0] += radian_noise * randn();
    C.camera_parameters_.angle_axis[1] += radian_noise * randn();
    C.camera_parameters_.angle_axis[2] += radian_noise * randn();
  }
}

void perturbPoints(vector<Point3dWithHistory>& points, double position_noise)
{
  double pos_noise = position_noise / sqrt(2.274625);
  BOOST_FOREACH (Point3dWithHistory& P, points)
  {
    P.point.x += pos_noise * randn();
    P.point.y += pos_noise * randn();
    P.point.z += pos_noise * randn();
  }
}

void independentlyPerturbCameras(vector<CameraWithHistory>& cameras)
{
  BOOST_FOREACH (CameraWithHistory& C, cameras)
  {
    double pos_noise = C.pose_position_sigma / sqrt(2.274625);
    double radian_noise = C.pose_orientation_sigma / sqrt(2.58047);
    C.camera_parameters_.position[0] += pos_noise * randn();
    C.camera_parameters_.position[1] += pos_noise * randn();
    C.camera_parameters_.position[2] += pos_noise * randn();
    C.camera_parameters_.angle_axis[0] += radian_noise * randn();
    C.camera_parameters_.angle_axis[1] += radian_noise * randn();
    C.camera_parameters_.angle_axis[2] += radian_noise * randn();
  }
}

void copyCamerasWoHistory(vector<CameraWithHistory>& original_cameras, vector<CameraWithHistory>& cameras)
{
  cameras.clear();
  BOOST_FOREACH (CameraWithHistory& C, original_cameras)
  {
    CameraWithHistory newC = C;
    newC.pose_history.clear();
    newC.num_observations = 0;
    cameras.push_back(newC);
  }
}

void copyPoints(vector<Point3dWithHistory>& original_points, vector<Point3dWithHistory>& points)
{
  points.clear();
  BOOST_FOREACH (Point3dWithHistory& P, original_points)
  {
    points.push_back(P);
  }
}

void addPoseToHistory(vector<CameraWithHistory>& cameras, vector<CameraWithHistory>& original_cameras)
{
  if (cameras.size() != original_cameras.size()) ROS_ERROR_STREAM("number of cameras in vectors do not match");

  for (int i = 0; i < (int)cameras.size(); i++)
  {
    Pose6d pose;
    pose.x = cameras[i].camera_parameters_.position[0];
    pose.y = cameras[i].camera_parameters_.position[1];
    pose.z = cameras[i].camera_parameters_.position[2];
    pose.ax = cameras[i].camera_parameters_.angle_axis[0];
    pose.ay = cameras[i].camera_parameters_.angle_axis[1];
    pose.az = cameras[i].camera_parameters_.angle_axis[2];
    original_cameras[i].pose_history.push_back(pose);
  }
}

void computeHistoricPoseStatistics(vector<CameraWithHistory>& cameras)
{
  // calculate statistics of test case
  double mean_x, mean_y, mean_z, mean_ax, mean_ay, mean_az;
  double sigma_x, sigma_y, sigma_z, sigma_ax, sigma_ay, sigma_az;
  BOOST_FOREACH (CameraWithHistory& C, cameras)
  {
    mean_x = 0.0;
    mean_y = 0.0;
    mean_z = 0.0;
    mean_ax = 0.0;
    mean_ay = 0.0;
    mean_az = 0.0;
    BOOST_FOREACH (Pose6d& p, C.pose_history)
    {
      mean_x += p.x;
      mean_y += p.y;
      mean_z += p.z;
      mean_ax += p.ax;
      mean_ay += p.ay;
      mean_az += p.az;
    }
    int num_poses = (int)C.pose_history.size();
    mean_x = mean_x / num_poses;
    mean_y = mean_y / num_poses;
    mean_z = mean_z / num_poses;
    mean_ax = mean_ax / num_poses;
    mean_ay = mean_ay / num_poses;
    mean_az = mean_az / num_poses;

    double d_x = mean_x - C.camera_parameters_.position[0];
    double d_y = mean_y - C.camera_parameters_.position[1];
    double d_z = mean_z - C.camera_parameters_.position[2];
    C.pose_position_mean_error = sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
    double d_ax = mean_ax - C.camera_parameters_.angle_axis[0];
    double d_ay = mean_ay - C.camera_parameters_.angle_axis[1];
    double d_az = mean_az - C.camera_parameters_.angle_axis[2];
    C.pose_orientation_mean_error = sqrt(d_ax * d_ax + d_ay * d_ay + d_az * d_az);

    sigma_x = 0.0;
    sigma_y = 0.0;
    sigma_z = 0.0;
    sigma_ax = 0.0;
    sigma_ay = 0.0;
    sigma_az = 0.0;
    BOOST_FOREACH (Pose6d& p, C.pose_history)
    {
      sigma_x += (mean_x - p.x) * (mean_x - p.x);
      ;
      sigma_y += (mean_y - p.y) * (mean_y - p.y);
      ;
      sigma_z += (mean_z - p.z) * (mean_z - p.z);
      ;
      sigma_ax += (mean_ax - p.ax) * (mean_ax - p.ax);
      ;
      sigma_ay += (mean_ay - p.ay) * (mean_ay - p.ay);
      ;
      sigma_az += (mean_az - p.az) * (mean_az - p.az);
      ;
    }
    sigma_x = sqrt(sigma_x / (num_poses - 1.0));
    sigma_y = sqrt(sigma_y / (num_poses - 1.0));
    sigma_z = sqrt(sigma_z / (num_poses - 1.0));
    C.sigma_ax = sigma_ax = sqrt(sigma_ax / (num_poses - 1.0));
    C.sigma_ay = sigma_ay = sqrt(sigma_ay / (num_poses - 1.0));
    C.sigma_az = sigma_az = sqrt(sigma_az / (num_poses - 1.0));

    double dv = sqrt(sigma_x * sigma_x + sigma_y * sigma_y + sigma_z * sigma_z);
    double av = sqrt(sigma_ax * sigma_ax + sigma_ay * sigma_ay + sigma_az * sigma_az);

    C.pose_position_sigma = dv;
    C.pose_orientation_sigma = av;
  }  // end for each camera
}

void computeHistoricPointStatistics(vector<Point3dWithHistory>& points)
{
  // calculate statistics of test case
  double mean_x, mean_y, mean_z;

  BOOST_FOREACH (Point3dWithHistory& P, points)
  {
    P.mean_x = P.mean_y = P.mean_z = 0.0;
    BOOST_FOREACH (double& p, P.x_history)
    {
      P.mean_x += p;
    }
    BOOST_FOREACH (double& p, P.y_history)
    {
      P.mean_y += p;
    }
    BOOST_FOREACH (double& p, P.z_history)
    {
      P.mean_z += p;
    }
    int num_values = (int)P.x_history.size();
    if (num_values == 0)
    {
      num_values = 1.0;
    }
    P.mean_x = P.mean_x / num_values;
    P.mean_y = P.mean_y / num_values;
    P.mean_z = P.mean_z / num_values;

    P.sigma_x = P.sigma_y = P.sigma_z = 0.0;
    BOOST_FOREACH (double& p, P.x_history)
    {
      P.sigma_x += (P.mean_x - p) * (P.mean_x - p);
      ;
    }
    BOOST_FOREACH (double& p, P.y_history)
    {
      P.sigma_y += (P.mean_y - p) * (P.mean_y - p);
      ;
    }
    BOOST_FOREACH (double& p, P.z_history)
    {
      P.sigma_z += (P.mean_z - p) * (P.mean_z - p);
      ;
    }
    double denom = num_values - 1.0;
    if (denom == 0 || P.num_observations == 0)
    {
      P.sigma_x = 1000;
      P.sigma_y = 1000;
      P.sigma_z = 1000;
    }
    else if (P.num_observations == 1)
    {  // can't triangulate with only one observation
      P.sigma_x = 0.4;
      P.sigma_y = 0.4;
      P.sigma_z = 0.4;
    }
    else
    {
      P.sigma_x = sqrt(P.sigma_x / denom);
      P.sigma_y = sqrt(P.sigma_y / denom);
      P.sigma_z = sqrt(P.sigma_z / denom);
    }
  }  // end for each point
}

void compareCameras(vector<CameraWithHistory>& C1, vector<CameraWithHistory>& C2)
{
  if (C1.size() != C2.size())
  {
    ROS_ERROR_STREAM("compareCameras() camera vectors different in length");
  }
  double d_x, d_y, d_z, dist;
  double mean_dist = 0;
  double sigma_dist = 0;
  double d_ax, d_ay, d_az, angle;
  double mean_angle = 0;
  double sigma_angle = 0;
  for (int i = 0; i < (int)C1.size(); i++)
  {
    if (C1[i].camera_name_ == C2[i].camera_name_)
    {
      d_x = C1[i].camera_parameters_.position[0] - C2[i].camera_parameters_.position[0];
      d_y = C1[i].camera_parameters_.position[1] - C2[i].camera_parameters_.position[1];
      d_z = C1[i].camera_parameters_.position[2] - C2[i].camera_parameters_.position[2];
      d_ax = C1[i].camera_parameters_.angle_axis[0] - C2[i].camera_parameters_.angle_axis[0];
      d_ay = C1[i].camera_parameters_.angle_axis[1] - C2[i].camera_parameters_.angle_axis[1];
      d_az = C1[i].camera_parameters_.angle_axis[2] - C2[i].camera_parameters_.angle_axis[2];
      dist = sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
      angle = sqrt(d_ax * d_ax + d_ay * d_ay + d_az * d_az);
      ROS_INFO("camera %s position_diff = %7.5lf meters, orientation diff = %7.5lf degrees", C1[i].camera_name_.c_str(),
               dist, angle * 180 / 3.14);
      mean_dist += dist;
      mean_angle += angle;
    }
    else
    {
      printf("cameras not same\n");
    }
  }
  mean_dist = mean_dist / (int)C1.size();
  mean_angle = mean_angle / (int)C1.size();
  for (int i = 0; i < (int)C1.size(); i++)
  {
    d_x = C1[i].camera_parameters_.position[0] - C2[i].camera_parameters_.position[0];
    d_y = C1[i].camera_parameters_.position[1] - C2[i].camera_parameters_.position[1];
    d_z = C1[i].camera_parameters_.position[2] - C2[i].camera_parameters_.position[2];
    d_ax = C1[i].camera_parameters_.angle_axis[0] - C2[i].camera_parameters_.angle_axis[0];
    d_ay = C1[i].camera_parameters_.angle_axis[1] - C2[i].camera_parameters_.angle_axis[1];
    d_az = C1[i].camera_parameters_.angle_axis[2] - C2[i].camera_parameters_.angle_axis[2];
    dist = sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
    angle = sqrt(d_ax * d_ax + d_ay * d_ay + d_az * d_az);
    sigma_dist += (dist - mean_dist) * (dist - mean_dist);
    sigma_angle += (angle - mean_angle) * (angle - mean_angle);
  }
  sigma_dist = sqrt(sigma_dist / C1.size());
  sigma_angle = sqrt(sigma_angle / C1.size());
  ROS_INFO("mean distance between camera poses = %9.5lf meters sigma= %9.5lf", mean_dist, sigma_dist);
  ROS_INFO("mean angle between camera poses = %9.5lf degrees sigma= %9.5lf", mean_angle * 180 / 3.1415,
           sigma_angle * 180 / 3.1415);
}

void compareObservations(vector<ObservationDataPoint>& O1, vector<ObservationDataPoint>& O2)
{
  if (O1.size() != O2.size())
  {
    ROS_ERROR_STREAM("compareObservations() vectors different in length");
  }
  double d_x, d_y, dist;
  double mean_dist = 0.0;
  double sigma_dist = 0.0;
  int n_compared = 0;
  for (int i = 0; i < (int)O1.size(); i++)
  {
    if (O1[i].camera_name_ == O2[i].camera_name_ && O1[i].point_id_ == O2[i].point_id_)
    {
      d_x = O1[i].image_x_ - O2[i].image_x_;
      d_y = O1[i].image_y_ - O2[i].image_y_;
      dist = sqrt(d_x * d_x + d_y * d_y);
      mean_dist += dist;
      n_compared++;
    }
  }
  mean_dist = mean_dist / n_compared;
  for (int i = 0; i < (int)O1.size(); i++)
  {
    if (O1[i].camera_name_ == O2[i].camera_name_ && O1[i].point_id_ == O2[i].point_id_)
    {
      d_x = O1[i].image_x_ - O2[i].image_x_;
      d_y = O1[i].image_y_ - O2[i].image_y_;
      dist = sqrt(d_x * d_x + d_y * d_y);
      sigma_dist += (dist - mean_dist) * (dist - mean_dist);
    }
  }
  sigma_dist = sqrt(sigma_dist / n_compared);
  ROS_INFO("mean distance between observations = %9.5lf pixels sigma= %9.5lf ", mean_dist, sigma_dist);
}

void addPointsToHistory(vector<Point3dWithHistory>& points, vector<Point3dWithHistory>& original_points)
{
  if (points.size() != original_points.size()) ROS_ERROR_STREAM("number of points in vectors do not match");

  for (int i = 0; i < (int)points.size(); i++)
  {
    original_points[i].x_history.push_back(points[i].point.x);
    original_points[i].y_history.push_back(points[i].point.y);
    original_points[i].z_history.push_back(points[i].point.z);
  }
}

bool parseScenes(std::string& scene_file_name, std::vector<Scene>& scenes)
{
  FILE* fp = fopen(scene_file_name.c_str(), "r");
  if (fp == NULL)
  {
    ROS_ERROR("Can't open file %s", scene_file_name.c_str());
    return (false);
  }
  int scene_id = 10;
  scenes.clear();
  while (fscanf(fp, "scene_id = %d/n", &scene_id) == 1)
  {
    Scene temp_scene;
    temp_scene.scene_id = scene_id;
    tf::Matrix3x3 R;
    double x, y, z;
    char dum[100], dum2[100], dum3[100];
    double z1, z2, z3, z4;
    fscanf(fp, "%s %s %s %lf %lf %lf %lf;/n", dum, dum2, dum3, &R[0][0], &R[0][1], &R[0][2], &x);
    fscanf(fp, "%lf %lf %lf %lf;/n", &R[1][0], &R[1][1], &R[1][2], &y);
    fscanf(fp, "%lf %lf %lf %lf;/n", &R[2][0], &R[2][1], &R[2][2], &z);
    fscanf(fp, "%lf %lf %lf %lf];/n", &z1, &z2, &z3, &z4);  // gobble last row
    temp_scene.target_pose.setBasis(R);
    tf::Vector3 v(x, y, z);
    temp_scene.target_pose.setOrigin(v);
    temp_scene.camera_names.clear();
    char temp_char_data[100];
    fscanf(fp, "%s %s %s %s", dum, dum2, dum3, temp_char_data);
    temp_scene.camera_names.push_back(std::string(temp_char_data));
    bool done = false;
    do
    {
      fscanf(fp, "%s", temp_char_data);
      if (temp_char_data[0] == ']')
      {
        done = true;
        char c = 'a';
        while (c != '\n')
          fscanf(fp, "%c", &c);
      }
      else
      {
        temp_scene.camera_names.push_back(std::string(temp_char_data));
      }
    } while (!done);
    ROS_INFO("scene %d has %d cameras", (int)scene_id, (int)temp_scene.camera_names.size());
    scenes.push_back(temp_scene);
  }  // end while there are more scenes
  fclose(fp);
  return (true);
}

void show_scenes(vector<Scene>& scenes)
{
  BOOST_FOREACH (Scene S, scenes)
  {
    ROS_INFO("scene_id %d", S.scene_id);
    S.target_pose.show("target pose:");
    BOOST_FOREACH (string s, S.camera_names)
    {
      ROS_INFO("%s", s.c_str());
    }
  }
}
void perturbPose(Pose6d& pose, double position_noise, double degree_noise)
{
  double pos_noise = position_noise / sqrt(2.274625);
  double radian_noise = degree_noise * 3.1415 / (180.0 * sqrt(2.58047));
  pose.x += pos_noise * randn();
  pose.y += pos_noise * randn();
  pose.z += pos_noise * randn();
  pose.ax += radian_noise * randn();
  pose.ay += radian_noise * randn();
  pose.ay += radian_noise * randn();
}

void computeObservationsOfPoints(vector<CameraWithHistory>& cameras, vector<Point3dWithHistory>& points,
                                 Pose6d& target_pose, int scene_id, vector<ObservationDataPoint>& observations,
                                 double camera_position_noise, double camera_degree_noise, double image_noise)

{
  observations.clear();
  BOOST_FOREACH (CameraWithHistory& C, cameras)
  {
    int q = 0;
    BOOST_FOREACH (Point3dWithHistory& P, points)
    {
      bool good_observation;
      ObservationDataPoint obs =
          predictObservationOfPoint(C, P, target_pose, scene_id, q++, image_noise, good_observation);
      if (good_observation)
      {
        observations.push_back(obs);
        P.num_observations++;
      }
    }
  }
}
void computeObservationsFromScenes(vector<Scene>& scenes,               // pose of target, and which cameras observed it
                                   vector<CameraWithHistory>& cameras,  // list of cameras
                                   double target_pos_noise,  // perturb position of target by this amount then compute
                                   double target_degree_noise,  // peturb orientation of target by this amount then
                                                                // compute
                                   double image_noise,          // amount of noise to add to each observation
                                   vector<ObservationDataPoint>& observations,  // returned data
                                   vector<Point3dWithHistory>& points)          // returned target points
{
  observations.clear();
  BOOST_FOREACH (CameraWithHistory& C, cameras)
    C.num_observations = 0;
  double pnoise = image_noise / sqrt(1.58085);  // magic number found by trial and error
  int rows = 10;
  int cols = 10;
  double spacing = .025;
  points.clear();
  BOOST_FOREACH (Scene& S, scenes)
  {
    // add a new set of target points to the vector of all points
    int target_point_offset = points.size();  // points[target_point_offset] is the first of the new points being added
    // add any target position and pose noise to target's pose
    if (target_pos_noise > 0.0 || target_degree_noise > 0.0)
    {
      perturbPose(S.target_pose, target_pos_noise, target_degree_noise);
    }
    addTargetPoints(rows, cols, spacing, points, S.target_pose);  // added in world coordinates
    // for each camera in scene
    BOOST_FOREACH (string camera_name, S.camera_names)
    {
      CameraWithHistory* current_camera_ptr = NULL;
      for (int i = 0; i < cameras.size(); i++)
      {
        if (cameras[i].camera_name_ == camera_name)
        {
          current_camera_ptr = &(cameras[i]);
        }
      }
      if (current_camera_ptr == NULL)
      {
        ROS_ERROR("Couldn't find camera %s", camera_name.c_str());
      }
      for (int i = target_point_offset; i < (int)points.size(); i++)
      {  // for each new point
        bool good_observation;
        ObservationDataPoint obs = predictObservationOfPoint(*current_camera_ptr, points[i], S.target_pose, S.scene_id,
                                                             i, image_noise, good_observation);
        if (good_observation)
        {
          observations.push_back(obs);
          current_camera_ptr->num_observations++;
          points[i].num_observations++;
        }
      }
    }  // end for each camera in scene
  }    // end for each scene
}  // end

ObservationDataPoint predictObservationOfPoint(CameraWithHistory& C, Point3dWithHistory& P, Pose6d& target_pose,
                                               int scene_id, int point_id, double image_noise,
                                               bool& results_within_image)
{
  double pnoise = image_noise / sqrt(1.58085);  // magic number found by trial and error
  double fx, fy, cx, cy, k1, k2, k3, p1, p2;
  double aa[3], tx[3];
  double ox, oy;
  double world_point[3];
  std::string t_name("modified_circle_grid");
  Pose6d dummy_target_pose;
  Pose6d dummy_intermediate_pose;
  std::string cost_type_string("CameraReprjErrorPK");
  Cost_function cost_type = string2CostType(cost_type_string);

  extractCameraIntrinsics(C.camera_parameters_.pb_intrinsics, fx, fy, cx, cy, k1, k2, k3, p1, p2);
  aa[0] = C.camera_parameters_.angle_axis[0];
  aa[1] = C.camera_parameters_.angle_axis[1];
  aa[2] = C.camera_parameters_.angle_axis[2];
  tx[0] = C.camera_parameters_.position[0];
  tx[1] = C.camera_parameters_.position[1];
  tx[2] = C.camera_parameters_.position[2];
  transformPoint(aa, tx, P.point.pb, world_point);              // move to world coordinates
  projectPntNoDistortion(world_point, fx, fy, cx, cy, ox, oy);  // project point into camera's image plane
  ox += pnoise * randn();                                       // add observation noise
  oy += pnoise * randn();
  results_within_image = true;
  if (ox < 0.0 || ox > C.camera_parameters_.width || oy < 0.0 || oy > C.camera_parameters_.height)
  {
    results_within_image = false;
  }
  ObservationDataPoint obs(C.camera_name_,
                           t_name,  // target_name
                           2,       // target type
                           scene_id, C.camera_parameters_.pb_intrinsics, C.camera_parameters_.pb_extrinsics, point_id,
                           target_pose.pb_pose, P.point.pb, ox, oy, cost_type, dummy_intermediate_pose, 0.0);
  return (obs);
}

void addTargetPoints(int rows, int cols, double spacing, vector<Point3dWithHistory>& points, Pose6d& target_pose)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      double target_point[3];
      double world_point[3];
      target_point[0] = j * spacing;
      target_point[1] = (rows - 1 - i) * spacing;
      target_point[2] = 0.0;
      poseTransformPoint(target_pose, target_point, world_point);  // move to world coordinates
      Point3dWithHistory point;
      point.point.x = world_point[0];
      point.point.y = world_point[1];
      point.point.z = world_point[2];
      points.push_back(point);
    }
  }
}
