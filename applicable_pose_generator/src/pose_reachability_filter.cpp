#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <urdf_model/model.h>

#include <kdl/chainiksolverpos_lma.hpp>

#include "ros/ros.h"

#include "applicable_pose_generator/pose_reachability_filter.h"
#include <boost/shared_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>

namespace create_chain_take_pose_inverse_kinamatics
{


bool chain_creation::chain_Parse(Eigen::Affine3d ei_transform_to_check)
{
  urdf::Model Mymodel;
  ros::NodeHandle nh("~");
  std::string robot_urdf;
  if (!nh.getParam("robot_urdf", robot_urdf)){
    ROS_ERROR("did not set parameter robot_urdf");

  }
  if (!Mymodel.initFile(robot_urdf))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  //todo link to boost and then make shared
  ROS_INFO("Successfully parsed urdf file");
  urdf::ModelInterfaceConstSharedPtr model_ptr = boost::make_shared<urdf::Model>(Mymodel);
  tesseract::tesseract_ros::KDLChainKin CK;
  if(!CK.init(model_ptr,"base_link","tool0","Lawrence_Robot"))
  {
    ROS_ERROR("failed to initiat the chain");
    return false;
  }
  KDL::Tree robot_tree;
  kdl_parser::treeFromUrdfModel(*model_ptr, robot_tree);
  KDL::Chain robot_chain;
  robot_tree.getChain("base_link","tool0",robot_chain);
  KDL::ChainIkSolverPos_LMA solving_Ik (robot_chain);
  KDL::JntArray robot_joints(robot_chain.getNrOfJoints()); //
  KDL::JntArray return_joint_values(robot_chain.getNrOfJoints());

  //get transfor that we are trying to reach with robot

  KDL::Frame transform_goal_kdl;
  tf::transformEigenToKDL(ei_transform_to_check,transform_goal_kdl);
  ROS_INFO("%d",solving_Ik.CartToJnt(robot_joints,transform_goal_kdl,return_joint_values));
  if(!solving_Ik.CartToJnt(robot_joints,transform_goal_kdl,return_joint_values))
  {
    ROS_INFO("Reachable Pose Found");
    return true;
  }
  else
  {
    return false;
    ROS_INFO("Unreachable pose found");
  }
  return false;
}
}//end of namespace create_chain_take_pose_inverse_kinamatics


