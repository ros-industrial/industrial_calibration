#include <industrial_extrinsic_cal/Swri_IRD_license.h>
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
