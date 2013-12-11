/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef CERES_COSTS_UTILS_HPP_
#define CERES_COSTS_UTILS_HPP_

namespace industrial_extrinsic_cal {

  /* local prototypes of helper functions */

/*! \brief print a quaternion plus position as a homogeneous transform 
 *  \param qx quaternion x value
 *  \param qy quaternion y value
 *  \param qz quaternion z value
 *  \param qw quaternion w value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz);

/*! \brief print an angle axis transform as a homogeneous transform
 *  \param x angle axis x value
 *  \param y angle axis y value
 *  \param z angle axis z value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printAATasH(double x, double y, double z, double tx, double ty, double tz);
/*! \brief print angle axis to homogeneous transform inverse
 *  \param x angle axis x value
 *  \param y angle axis y value
 *  \param z angle axis z value
 *  \param tx position x value
 *  \param ty position y value
 *  \param tz position z value
 */
void printAATasHI(double x, double y, double z, double tx, double ty, double tz);
/*! \brief print angle axis as euler angles
 *  \param x angle axis x value
 *  \param y angle axis y value
 *  \param z angle axis z value
 */
void printAAasEuler(double x, double y, double z);
/*! \brief print Camera Parameters
 *  \param CameraParameters include intrinsic and extrinsic
 *  \param words to provide as a header
 */
void printCameraParameters(CameraParameters C, std::string words);

/*! \brief  computes image of point in cameras image plane 
 *  \param  C both intrinsic and extrinsic camera parameters
 *  \param  P the point to be projected into image
*/
observation projectPoint(CameraParameters C, Point3d P);

} // end of namespace
#endif
