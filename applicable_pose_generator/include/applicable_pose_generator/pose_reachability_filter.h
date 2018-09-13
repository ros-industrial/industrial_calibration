/*
    Software License Agreement (Apache License)
    Copyright (c) 2014, Southwest Research Institute
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#ifndef POSE_REACHABILITY_FILTER_H_
#define POSE_REACHABILITY_FILTER_H_


#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <tesseract_ros/kdl/kdl_chain_kin.h>

#include "ros/ros.h"

#include <boost/shared_ptr.hpp>

/*!
 * This class creates the chain of transforms from joint to joint for the robot
 * It then checks to see if that chain can reach the pose that we want
 */
namespace CreateChain
{
  class chain_creation
  {
    public:
    std::string robot_urdf_;
    std::string tool0_param_;
    std::string base_link_param_;
    //loads urdf that discribes the chain
    chain_creation();
    bool chain_Parse(Eigen::Affine3d ei_transform_to_check);
  private:
    KDL::JntArray robot_joints;
    KDL::JntArray return_joint_values;
    urdf::Model Mymodel;
    tesseract::tesseract_ros::KDLChainKin CK;
    KDL::Tree robot_tree;
    KDL::Chain robot_chain;
    KDL::Frame transform_goal_kdl;
  };
}
#endif
