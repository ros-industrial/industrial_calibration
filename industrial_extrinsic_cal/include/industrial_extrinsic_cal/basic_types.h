/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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

#ifndef BASIC_TYPES_H_
#define BASIC_TYPES_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <tf/LinearMath/Matrix3x3.h>
//#include <industrial_extrinsic_cal/basic_types.h>

namespace industrial_extrinsic_cal
{

typedef double* P_BLOCK;

/*! \brief A region of interest in an image */
typedef struct
{
  int x_min;
  int x_max;
  int y_min;
  int y_max;
} Roi;

/*! Brief Point3d defines a ceres_structure for a point in 3D space */
typedef struct
{
  union
  {
    struct
    {
      double x; /**< position x */
      double y; /**< position y */
      double z; /**< position z */
    };
    double pb[3]; /**< a parameter block with all three elements */
  };
} Point3d;

/*! Brief Pose6d defines a ceres_structure for a pose in 3D space 
 *   x,y,z have their natrual meanging
 *   ax,ay,az define the rotation part of the pose using angle axis notation
 */
class Pose6d
{
 public:
  Pose6d(double tx, double ty, double tz, double aax, double aay, double aaz);
  Pose6d();
  void set_basis( tf::Matrix3x3 m);
  void set_origin(tf::Vector3 v);
  void set_origin(double tx, double ty, double tz);
  void set_eulerZYX(double ez, double ey, double ex);
  void set_quaternion(double qx, double qy, double qz, double qw);
  void set_angle_axis(double aax, double aay, double aaz);
  tf::Matrix3x3 get_basis();
  tf::Vector3 get_origin();
  //TODO  void get_eulerZYX(double &ez, double &ey, double &ex);
  void get_quaternion(double &qx,  double &qy, double &qz, double &qw);

  union
  {
    struct
    {
      double x; /**< position x */
      double y; /**< position y */
      double z; /**< position z */
      double ax; /**< angle axis x value */
      double ay; /**< angle axis y value */
      double az; /**< angle axis z value */
    };
    struct
    {
      double pb_loc[3]; /**< parameter block for position */
      double pb_aa[3]; /**< parameter block for rotation */
    };
    struct
    {
      double pb_pose[6]; /**< a third option with a single block for 6dof pose */
    };
  };
};
  
/** @brief Parameters defining checker board target   */
typedef struct
{
  int pattern_rows;
  int pattern_cols;
} CheckerBoardParameters;
/** @brief Parameters defining circle grid target  */
typedef struct
{
  int pattern_rows;		// number of rows
  int pattern_cols;		// number of colulmns
  bool is_symmetric;		// not sure
  double circle_diameter;	// size of each circle
} CircleGridParameters;
/** @brief Parameters defining AR target */
typedef struct
{
  //std::string marker_pattern;
  double marker_width;
} ARTargetParameters;

/*! \brief A target's information */
typedef struct
{
  std::string target_name;
  unsigned int target_type;

  union
  {
    CheckerBoardParameters checker_board_parameters;
    CircleGridParameters circle_grid_parameters;
    ARTargetParameters ar_target_parameters;
  };
  bool is_moving; /**< observed in multiple locations or it fixed to ref frame */
  Pose6d pose;
  unsigned int num_points; /**< number of points in the point array */
  std::vector<Point3d> pts; /**< an array of points expressed relative to Pose p. */
  bool fixed_pose; /**< is the location of the target known? **/
  bool fixed_points; /**< are the locations of the points within the target known */
} Target;

/*! \brief An observation is the x,y image location of a target's point in an image*/
typedef struct
{
  boost::shared_ptr<Target> target; /**< pointer to target who's point is observed */
  int point_id; /**< point's id in target's point array */
  double image_loc_x; /**< target point was found at image location x */
  double image_loc_y; /**< target point image location y */
} Observation;

/*! \brief A vector of observations made by a single camera of posibly multiple targets */
typedef struct
{
  std::vector<Observation> observations;
} CameraObservations;

/*! Brief CameraParameters defines both the intrinsic and extrinsic parameters of a camera
 */
typedef struct
{
  union
  {
    struct
    {
      double angle_axis[3]; /**< angle axis data */
      double position[3]; /**< position data */
      double focal_length_x; /**< focal length in x */
      double focal_length_y; /**< focal length in y */
      double center_x; /**< central pixel x value */
      double center_y; /**< central pixel y value */
      double distortion_k1; /**< 2nd order radial distortion parameter */
      double distortion_k2; /**< 4th order radial distortion parameter */
      double distortion_k3; /**< 6th order radial distortion parameter */
      double distortion_p1; /**< 1st tangential distortion parameter */
      double distortion_p2; /**< 2nd tangential distortion parameter */
    };
    struct
    { /** parameter blocks for ceres */
      double pb_extrinsics[6]; /** parameter block for intrinsics */
      double pb_intrinsics[9]; /** parameter block for extrinsics */
    };
    struct
    {
      double pb_all[15]; /** parameter block for both */
    };
  };
} CameraParameters;

/*! @brief what kind of trigger initiates the collection of data for this scene */
class Trigger
{ /** Trigger */
 public:
  enum TRIGGER_TYPE{
    GRAB_NEXT_IMAGE,
    ROS_BOOL_PARAM,
    ACTION_TRIGGER
  };
  Trigger()
    {
      type_ = Trigger::GRAB_NEXT_IMAGE;
      ros_bool_param_ = "start_scene_capture";
      action_msg_ = "Set up next static scene ";
      action_srv_  = "scene_trigger_action_server";
    }
  Trigger(Trigger::TRIGGER_TYPE tt,
	  std::string msg="HIT RETURN TO ACCEPT IMAGE")
    {
      type_ = tt;
      action_msg_ = msg;
      ros_bool_param_ = "start_scene_capture";
      action_srv_  = "scene_trigger_action_server";
    };
  ~Trigger(){};
  void setTriggerType( Trigger::TRIGGER_TYPE tt) { type_ = tt; };
  void setRosBoolParam( std::string msg) { ros_bool_param_ = msg; };
  void setActionMessage( std::string msg) { action_msg_ = msg; };
  void setActionServer( std::string msg) { action_msg_ = msg; };
  Trigger::TRIGGER_TYPE  getTriggerType( ) { return (Trigger::TRIGGER_TYPE)type_; };
  std::string  getRosBoolParam( ) { return ros_bool_param_; };
  std::string  getActionMessage( ) { return action_msg_; };
  std::string  getActionServer( ) { return action_srv_; };
 private:
  int type_;
  std::string ros_bool_param_;
  std::string action_msg_;
  std::string action_srv_;
} ;

/*! \brief moving  need a new pose with each scene in which they are used */
typedef struct MovingTarget
{
  boost::shared_ptr<Target> targ; // must hold a copy of the target pose parameters,
  int scene_id; // but point parameters may be unused duplicates
} MovingTarget;

}//end namespace industrial_extrinsiac_cal
#endif
