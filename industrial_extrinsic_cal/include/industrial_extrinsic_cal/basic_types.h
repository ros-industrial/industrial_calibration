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

#ifndef BASIC_TYPES_H_
#define BASIC_TYPES_H_

#include <boost/shared_ptr>

namespace industrial_extrinsic_cal {

/*! \brief A region of interest in an image */
typedef struct{
  int x_min;
  int x_max;
  int y_min;
  int y_max;
} Roi;

/*! \brief A target's information */
typedef struct{
  std::string target_name;
  std::target_type;
  pose p;
  std::vector<Point_3d> pts; /** an array of points expressed relative to Pose p. */
  bool is_moving;  /** observed in multiple locations or it fixed to ref frame */
  bool pose_free;  /** is the pose a parameter or not? */
  bool pts_x_free; /** is x a free parameter? generally only one of Pose or (xyz) is free */
  bool pts_y_free; /** is y a free parameter? */
  bool pts_z_free; /** is z a free parameter? */
} Target;

/*! \brief An observations information to be filled out by the observeing camera*/
typedef struct { /** image of a point */
  boost::shared_ptr<Target> T;	/* pointer to target structure */
  int p_id;			/** point's id in target's point array */
  double x;			/** image x */
  double y;			/** image y */
}Observation;

}
#endif
