#include <kdl/chainiksolverpos_lma.hpp>

#include "ros/ros.h"

#include <boost/shared_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>

namespace create_chain_take_pose_inverse_kinamatics
{
  class chain_creation
  {
    public:
      //loads urdf that discribes the chain
    bool chain_Parse(Eigen::Affine3d ei_transform_to_check);
    //bool reachability_filter();
  };
}
