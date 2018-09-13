#include <urdf_model/model.h>

#include <kdl/chainiksolverpos_lma.hpp>

#include "ros/ros.h"

#include "applicable_pose_generator/pose_reachability_filter.h"
#include <boost/shared_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>


namespace CreateChain
{
chain_creation::chain_creation()
{
  ros::NodeHandle pnh("~");
  if (!pnh.getParam("robot_urdf", robot_urdf_)){
    ROS_ERROR("did not set parameter robot_urdf");
    exit (EXIT_FAILURE);
  }
  if (!pnh.getParam("base_link", base_link_param_)){
    ROS_ERROR("did not set parameter base_link");
    exit (EXIT_FAILURE);
  }
  if (!pnh.getParam("tool0", tool0_param_)){
    ROS_ERROR("did not set parameter tool0");
    exit (EXIT_FAILURE);
  }
  if (!Mymodel.initFile(robot_urdf_))
  {
    ROS_ERROR("Failed to parse urdf file");
    return;
  }
  else
  {
  ROS_INFO("Successfully parsed urdf file");
  }
  urdf::ModelInterfaceConstSharedPtr model_ptr = boost::make_shared<urdf::Model>(Mymodel);
  if(!CK.init(model_ptr,base_link_param_,tool0_param_,"Robot_from_urdf"))
  {
    ROS_ERROR("failed to initiat the chain");
    return;
  }
  kdl_parser::treeFromUrdfModel(*model_ptr, robot_tree);
  robot_tree.getChain(base_link_param_,tool0_param_,robot_chain);
}

bool chain_creation::chain_Parse(Eigen::Affine3d ei_transform_to_check)
{
  //get transform that we are trying to reach with robot
  KDL::ChainIkSolverPos_LMA solving_Ik(robot_chain);
  robot_joints.resize(robot_chain.getNrOfJoints());
  return_joint_values.resize(robot_chain.getNrOfJoints());
  tf::transformEigenToKDL(ei_transform_to_check,transform_goal_kdl);

  ROS_INFO("%d",solving_Ik.CartToJnt(robot_joints,transform_goal_kdl,return_joint_values));

  if(!solving_Ik.CartToJnt(robot_joints,transform_goal_kdl,return_joint_values))
  {
    ROS_INFO("Reachable Pose Found");
    return true;
  }
  else
  {
    ROS_INFO("Unreachable pose found");
    return false;
  }
}
}//end of namespace create_chain_take_pose_inverse_kinamatics


