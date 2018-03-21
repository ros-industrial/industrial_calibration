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
#include <limits>
#include <ceres/rotation.h>

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
  /** @brief constructor with complete information
   *    @param tx translation in x
   *    @param ty translation in y
   *    @param tz translation in z
   *    @param aax x component of angle axis vector
   *    @param aay y component of angle axis vector
   *    @param aaz z component of angle axis vector
  */
  Pose6d(double tx, double ty, double tz, double aax, double aay, double aaz);

  /** @brief default constructor*/
  Pose6d();

  /** @brief set the rotational part of pose using a tf::Matrix3x3
   *    @param m a 3x3 matrix representing the rotation
   */
  void setBasis(tf::Matrix3x3& m);

  /** @brief set the translational part of pose using a tf::Vector3
   *    @param v the translation components as a 3 vector
  */
  void setOrigin(tf::Vector3& v);

  /** @brief set the translational part of pose
   *    @param tx  the x value of the translation vector
   *    @param ty  the y value of the translation vector
   *    @param tz  the z value of the translation vector
  */
  void setOrigin(double tx, double ty, double tz);

  /** @brief set the rotational part of pose using Euler Z-Y-Z rotations
   *   @param  ez Rotation angle around Z axis
   *   @param  ey Rotation angle around y axis
   *   @param  ex Rotation angle around x axis
   */
  void setEulerZYX(double ez, double ey, double ex);

  /** @brief set the rotational part of pose using a quaternion
   *   @param qx quaternion x value
   *   @param qy quaternion y value
   *   @param qz quaternion z value
   *   @param qw quaternion w value
   */
  void setQuaternion(double qx, double qy, double qz, double qw);

  /** @brief set the rotational part of pose using the angle axis notation
   *    @param aax x component of angle axis vector
   *    @param aay y component of angle axis vector
   *    @param aaz z component of angle axis vector
  */
  void setAngleAxis(double aax, double aay, double aaz);

  /** @brief get the rotational part of pose as a tf::Matrix3x3 */
  tf::Matrix3x3 getBasis() const;

  /** @brief get the euler angles
   * @param ez angle of rotation around z axis
   * @param ey angle of rotation around y axis
   * @param ex angle of rotation around x axis
   */
  void getEulerZYX(double& ez, double& ey, double& ex) const;

  /** @brief get the translationalpart of pose as a tf::Vector3*/
  tf::Vector3 getOrigin() const;

  // TODO  void get_eulerZYX(double &ez, double &ey, double &ex);

  /** @brief get the translationalpart of pose as a quaternion
   *   @param qx quaternion x value
   *   @param qy quaternion y value
   *   @param qz quaternion z value
   *   @param qw quaternion w value
  */
  void getQuaternion(double& qx, double& qy, double& qz, double& qw) const;

  /** @brief get the inverse of the pose_*/
  Pose6d getInverse() const;

  /** @brief output pose info
   *    @param message to display along with pose info
   */
  void show(std::string message);

  /** @brief multiplication operator*/
  Pose6d operator*(Pose6d pose2) const;
  union
  {
    struct
    {
      double ax; /**< angle axis x value */
      double ay; /**< angle axis y value */
      double az; /**< angle axis z value */
      double x;  /**< position x */
      double y;  /**< position y */
      double z;  /**< position z */
    };
    struct
    {
      double pb_aa[3];  /**< parameter block for rotation */
      double pb_loc[3]; /**< parameter block for position */
    };
    struct
    {
      double pb_pose[6]; /**< a third option with a single block for 6dof pose */
    };
  };  // end of union
};    // end of class Pose6d

/** @brief Parameters defining checker board target   */
typedef struct
{
  int pattern_rows;
  int pattern_cols;
  double square_size;
  double spacing;
} CheckerBoardParameters;

/** @brief Parameters defining circle grid target  */
typedef struct
{
  int pattern_rows;        // number of rows
  int pattern_cols;        // number of colulmns
  bool is_symmetric;       // not sure
  double circle_diameter;  // size of each circle
  double spacing;          // spacing between circle centers
} CircleGridParameters;

/** @brief Parameters defining AR target */
typedef struct
{
  double marker_width;
} ARTargetParameters;

/*! Brief CameraParameters defines both the intrinsic and extrinsic parameters of a camera
 */
typedef struct
{
  union
  {
    struct
    {
      double angle_axis[3];  /**< angle axis data */
      double position[3];    /**< position data */
      double focal_length_x; /**< focal length in x */
      double focal_length_y; /**< focal length in y */
      double center_x;       /**< central pixel x value */
      double center_y;       /**< central pixel y value */
      double distortion_k1;  /**< 2nd order radial distortion parameter */
      double distortion_k2;  /**< 4th order radial distortion parameter */
      double distortion_k3;  /**< 6th order radial distortion parameter */
      double distortion_p1;  /**< 1st tangential distortion parameter */
      double distortion_p2;  /**< 2nd tangential distortion parameter */
    };
    struct
    {                          /** parameter blocks for ceres */
      double pb_extrinsics[6]; /** parameter block for intrinsics */
      double pb_intrinsics[9]; /** parameter block for extrinsics */
    };
    struct
    {
      double pb_all[15]; /** parameter block for both */
    };
  };  // end of union
  int height;
  int width;
} CameraParameters;

}  // end namespace industrial_extrinsiac_cal
#endif
