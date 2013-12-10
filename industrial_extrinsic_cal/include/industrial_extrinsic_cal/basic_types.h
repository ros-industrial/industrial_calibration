typedef struct{
  int x_min;
  int x_max;
  int y_min;
  int y_max;
} Roi;

/** these structures define observation data collected */
typedef struct { /** image of a point */
  Target *T;			/* pointer to target structure */
  int p_id;			/** point's id in target's point array */
  double x;			/** image x */
  double y;			/** image y */
}Observation;

typedef struct{ /** a target's information */
  std::string target_name;
  std::target_type;
  pose P;
  std::vector<Point_3d> pts; /** an array of points expressed relative to Pose P. */
  bool is_moving;  /** observed in multiple locations or it fixed to ref frame */
  bool pose_free;  /** is the pose a parameter or not? */
  bool pts_x_free; /** is x a free parameter? generally only one of Pose or (xyz) is free */
  bool pts_y_free; /** is y a free parameter? */
  bool pts_z_free; /** is z a free parameter? */
} Target;
