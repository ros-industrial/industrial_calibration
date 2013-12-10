
/** structures for ceres */
typedef struct { /** point in 3D space */
  union{
    struct{ /** parameters for coding */
      double x;
      double y;
      double z;
    };
    double PB[3];		/** parameter block for ceres */
  };
}Point3d;

typedef struct { /** 6D pose using angle axis notation */
  union{
    struct{
      double x; /** parameters for coding */
      double y;
      double z;
      double ax;
      double ay;
      double az;
    };
    struct{		/** parameter blocks for ceres */
      double PB_loc[3];
      double PB_aa[3];
    };
  };
}Pose6d;

typedef struct{ /** camera parameters */
  union{
    struct{ /** parameters for coding */
      double offset_aa[3];            /** angle axis data for offset */
      double offset_pos[3];           /** positions axis data for offset */
      double aa[3];		      /** angle axis data */
      double pos[3];		      /** position data */
      double fx;
      double fy;
      double cx;
      double cy;
      double k1;
      double k2;
      double k3;
      double p1;
      double p2;
    };
    struct{ /** parameter blocks for ceres */
      double PB_extrinsics_offset[6]; /** offset transform of fixed parameters */
      double PB_extrinsics[6];	      /** parameter block for intrinsics */
      double PB_intrinsics[9];        /** parameter block for extrinsics */
    };

  };
}CameraParameters;

