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
#ifndef CERES_EXCAL_HPP_
#define CERES_EXCAL_HPP_
namespace industrial_extrinsic_cal {

/*! \brief ceres_excal.h defines structures for ceres
 *   These structures are all unions of two things
 *   The first is a set of meaningful variables (x,y,z,ax,ay,az describe a pose)
 *   The second is a parameter block which overlays all the variables
 *   A pointer to the second block is passed to the templated cost function
 *   which must use these variables to compute the cost, but can't accept arbitrary 
 *   but programmer friendly structures.
 *   In some cases a third parameter block is also supplied which groups multiple blocks
 *   together further reducing the number of parameters pointers sent to a cost function
 */

/*! Brief CameraParameters defines both the intrinsic and extrinsic parameters of a camera
 */
typedef struct{ 
  union{
    struct{ 
      double aa[3];		      /** angle axis data */
      double pos[3];		      /** position data */
      double fx;		      /** focal length in x */
      double fy;		      /** focal length in y */
      double cx;                      /** central pixel x value */
      double cy;                      /** central pixel y value */
      double k1;                      /** 2nd order radial distortion parameter */
      double k2;                      /** 4th order radial distortion parameter */
      double k3;                      /** 6th order radial distortion parameter */
      double p1;                      /** 1st tangential distortion parameter */
      double p2;                      /** 2nd tangential distortion parameter */
    };
    struct{ /** parameter blocks for ceres */
      double pb_extrinsics[6];	      /** parameter block for intrinsics */
      double pb_intrinsics[9];        /** parameter block for extrinsics */
    };
    struct{
      double pb_all[15];         /** parameter block for both */
    };
  };
}CameraParameters;

/*! Brief CameraExParameters defines the extrinsic parameters of a camera
 *  This allows a reduced set of parameters to be sent to a simpler cost function
 *  Here it is assumed that the rectified image is being used 
 */
typedef struct{ 
  union{
    struct{
      double x;			/** position x */ 
      double y;			/** position y */ 
      double z;			/** position z */ 
      double ax;		/** angle axis x value */ 
      double ay;		/** angle axis y value */ 
      double az;		/** angle axis z value */ 
    };
    struct{ 
      double pos[3];		      /** position data */
      double aa[3];		      /** angle axis data */
    };
    struct{ /** parameter blocks for ceres */
      double pb_extrinsics[6];	      /** parameter block for intrinsics */
    };
  };
}CameraExParameters;

/*! \brief this class keeps the ceres parameter blocks for a target as a single unit*/
class TargetParameters{
public:
  /*! \brief constructor (must know how many points are on target) 
   *  \param number_of_points The number of key-points on the target */
  TargetParameters(int number_of_points){
    pts_.resize(number_of_points);
  };
  /*! \brief gets the pose parameter block for ceres */ 
  double* getPoseParameterBlock() { return(pose_.pb); };

  /*! \brief gets the point parameter block for ceres */ 
  double* getPointParameterBlock( unsigned int i){
    if(i<(unsigned int) pts_.size()) return(pts_[i].pb);
    return(NULL); 
  };
  /*! \brief sets the location of a point */ 
  bool set_point(Point3d input_point, unsigned int point_id){
    if(point_id>(unsigned int) pts.size()) return(false);
    pts_[i].x = input_point.x;
    pts_[i].y = input_point.y;
    pts_[i].z = input_point.z;
  };
private:  
  Pose6d pose_;
  std::vector<Point3d> pts_;
}TargetParameters;

} \\end of namespace

#endif
