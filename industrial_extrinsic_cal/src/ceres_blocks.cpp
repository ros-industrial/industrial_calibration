/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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


#include <industrial_extrinsic_cal/ceres_blocks.h>
#include <boost/shared_ptr.hpp>

using std::string;
using boost::shared_ptr;

namespace industrial_extrinsic_cal
{
CeresBlocks::CeresBlocks()
{
}
CeresBlocks::~CeresBlocks()
{
  clearCamerasTargets();
}
void CeresBlocks::clearCamerasTargets()
{
  //ROS_INFO_STREAM("Attempting to clear cameras and targets from ceresBlocks");
  static_targets_.clear();
  //ROS_INFO_STREAM("Moving Targets "<<moving_targets_.size());
  moving_targets_.clear();
  //ROS_INFO_STREAM("Static cameras "<<static_cameras_.size());
  static_cameras_.clear();
  //ROS_INFO_STREAM("Moving cameras "<<moving_cameras_.size());
  moving_cameras_.clear();
  //ROS_INFO_STREAM("Cameras and Targets cleared from CeresBlocks");
}
P_BLOCK CeresBlocks::getStaticCameraParameterBlockIntrinsics(string camera_name)
{
  // static cameras should have unique name
  BOOST_FOREACH(shared_ptr<Camera> camera, static_cameras_)
  {
    if (camera_name == camera->camera_name_)
    {
      P_BLOCK intrinsics = &(camera->camera_parameters_.pb_intrinsics[0]);
      return (intrinsics);
    }
  }
  return (NULL);
}
P_BLOCK CeresBlocks::getMovingCameraParameterBlockIntrinsics(string camera_name)
{
  // we use the intrinsic parameters from the first time the camera appears in the list
  // subsequent cameras with this name also have intrinsic parameters, but these are
  // never used as parameter blocks, only their extrinsics are used
  BOOST_FOREACH(shared_ptr<MovingCamera> moving_camera, moving_cameras_)
  {
    if (camera_name == moving_camera->cam->camera_name_)
    {
      P_BLOCK intrinsics = &(moving_camera->cam->camera_parameters_.pb_intrinsics[0]);
      return (intrinsics);
    }
  }
  return (NULL);
}
P_BLOCK CeresBlocks::getStaticCameraParameterBlockExtrinsics(string camera_name)
{
  // static cameras should have unique name
  BOOST_FOREACH(shared_ptr<Camera> camera, static_cameras_)
  {
    if (camera_name == camera->camera_name_)
    {
      P_BLOCK extrinsics = &(camera->camera_parameters_.pb_extrinsics[0]);
      return (extrinsics);
    }
  }
  return (NULL);

}
P_BLOCK CeresBlocks::getMovingCameraParameterBlockExtrinsics(string camera_name, int scene_id)
{
  BOOST_FOREACH(shared_ptr<MovingCamera> camera, moving_cameras_)
  {
    if (camera_name == camera->cam->camera_name_ && scene_id == camera->scene_id)
    {
      P_BLOCK extrinsics = &(camera->cam->camera_parameters_.pb_extrinsics[0]);
      return (extrinsics);
    }
  }
  return (NULL);

}
P_BLOCK CeresBlocks::getStaticTargetPoseParameterBlock(string target_name)
{
  BOOST_FOREACH(shared_ptr<Target> target, static_targets_)
  {
    if (target_name == target->target_name_)
    {
      P_BLOCK pose = &(target->pose_.pb_pose[0]);
      return (pose);
    }
  }
  return (NULL);
}
P_BLOCK CeresBlocks::getStaticTargetPointParameterBlock(string target_name, int point_id)
{
  BOOST_FOREACH(shared_ptr<Target> target, static_targets_)
  {
    if (target_name == target->target_name_)
    {
      P_BLOCK point_position = &(target->pts_[point_id].pb[0]);
      return (point_position);
    }
  }
  return (NULL);
}
P_BLOCK CeresBlocks::getMovingTargetPoseParameterBlock(string target_name, int scene_id)
{
  BOOST_FOREACH(shared_ptr<MovingTarget> moving_target, moving_targets_)
  {
    if (target_name == moving_target->targ_->target_name_ && scene_id == moving_target->scene_id_)
    {
      P_BLOCK pose = &(moving_target->targ_->pose_.pb_pose[0]);
      return (pose);
    }
  }
  return (NULL);
}
P_BLOCK CeresBlocks::getMovingTargetPointParameterBlock(string target_name, int pnt_id)
{
  // note scene_id unnecessary here since regarless of scene th point's location relative to
  // the target frame does not change
  BOOST_FOREACH(shared_ptr<MovingTarget> moving_target, moving_targets_)
  {
    if (target_name == moving_target->targ_->target_name_)
    {
      P_BLOCK point_position = &(moving_target->targ_->pts_[pnt_id].pb[0]);
      return (point_position);
    }
  }
  return (NULL);
}

bool CeresBlocks::addStaticCamera(shared_ptr<Camera> camera_to_add)
{
  BOOST_FOREACH(shared_ptr<Camera> cam, static_cameras_)
  {
    if (cam->camera_name_ == camera_to_add->camera_name_)
      return (false); // camera already exists
  }
  camera_to_add->setTIReferenceFrame(reference_frame_);
  static_cameras_.push_back(camera_to_add);
  //ROS_INFO_STREAM("Camera added to static_cameras_");
  return (true);
}
bool CeresBlocks::addStaticTarget(shared_ptr<Target> target_to_add)
{
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
  {
    if (targ->target_name_ == target_to_add->target_name_)
      {
	return (false); // target already exists
      }
  }
  target_to_add->setTIReferenceFrame(reference_frame_);
  static_targets_.push_back(target_to_add);

  return (true);
}
bool CeresBlocks::addMovingCamera(shared_ptr<Camera> camera_to_add, int scene_id)
{
  BOOST_FOREACH(shared_ptr<MovingCamera> cam, moving_cameras_)
  {
    if (cam->cam->camera_name_ == camera_to_add->camera_name_ && cam->scene_id == scene_id)
      return (false); // camera already exists
  }
  // this next line allocates the memory for a moving camera
  shared_ptr<MovingCamera> temp_moving_camera = boost::make_shared<MovingCamera>();
  // this next line allocates the memory for the actual camera
  shared_ptr<Camera> temp_camera = boost::make_shared<Camera>(camera_to_add->camera_name_, camera_to_add->camera_parameters_,
                                                       true);
  temp_moving_camera->cam = temp_camera;
  temp_moving_camera->scene_id = scene_id;
  temp_moving_camera->cam->setTIReferenceFrame(reference_frame_);
  moving_cameras_.push_back(temp_moving_camera);
  return (true);
}
bool CeresBlocks::addMovingTarget(shared_ptr<Target> target_to_add, int scene_id)
{
  BOOST_FOREACH(shared_ptr<MovingTarget> targ, moving_targets_)
  {
    if (targ->targ_->target_name_ == target_to_add->target_name_ && targ->scene_id_ == scene_id)
      return (false); // target already exists
  }
  shared_ptr<MovingTarget> temp_moving_target = boost::make_shared<MovingTarget>();
  shared_ptr<Target> temp_camera = boost::make_shared<Target>();
  temp_moving_target->targ_ = target_to_add;
  temp_moving_target->scene_id_ = scene_id;
  temp_moving_target->targ_->setTIReferenceFrame(reference_frame_);
  moving_targets_.push_back(temp_moving_target);
  return (true);
}

const boost::shared_ptr<Camera> CeresBlocks::getCameraByName(const std::string &camera_name)
{
  boost::shared_ptr<Camera> cam = boost::make_shared<Camera>();
  //ROS_INFO_STREAM("Found "<<static_cameras_.size() <<" static cameras");
  for (int i=0; i< static_cameras_.size() ; i++ )
  {
    if (static_cameras_.at(i)->camera_name_==camera_name)
    {
      cam= static_cameras_.at(i);
      ROS_DEBUG_STREAM("Found static camera with name: "<<static_cameras_.at(i)->camera_name_);
    }
  }
  //ROS_INFO_STREAM("Found "<<moving_cameras_.size() <<" moving cameras");
  for (int i=0; i< moving_cameras_.size() ; i++ )
  {
    if (moving_cameras_.at(i)->cam->camera_name_==camera_name)
    {
      cam= moving_cameras_.at(i)->cam;
      ROS_DEBUG_STREAM("Found moving camera with name: "<<camera_name);
    }
  }
  if (!cam)
  {
    ROS_ERROR_STREAM("Fail");
  }
  return cam;
  //return true;
}

const boost::shared_ptr<Target> CeresBlocks::getTargetByName(const std::string &target_name)
{
  boost::shared_ptr<Target> target = boost::make_shared<Target>();
  bool found=false;
  //ROS_INFO_STREAM("Found "<<static_cameras_.size() <<" static cameras");
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
  {
    if (targ->target_name_==target_name)
    {
      target=targ;
      found = true;
      ROS_DEBUG_STREAM("Found static target with name: "<<target_name);
    }
  }
  //ROS_INFO_STREAM("Found "<<moving_cameras_.size() <<" static cameras");
  BOOST_FOREACH(shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    if (mtarg->targ_->target_name_ ==target_name)
    {
      found = true;
      target=mtarg->targ_;
      ROS_DEBUG_STREAM("Found moving target with name: "<<target_name);
    }
  }
  if (!found)
  {
    ROS_ERROR_STREAM("Fail");
  }
  return target;
}


 void CeresBlocks::displayStaticCameras()
{
  double R[9];
  double aa[3];
  double camera_to_world[3];
  double world_to_camera[3];
  double quat[4];

  if(static_cameras_.size() !=0)   ROS_INFO("Static Cameras");
  BOOST_FOREACH(shared_ptr<Camera> cam, static_cameras_)
    {
      aa[0] = -cam->camera_parameters_.angle_axis[0];
      aa[1] = -cam->camera_parameters_.angle_axis[1];
      aa[2] = -cam->camera_parameters_.angle_axis[2];
      camera_to_world[0] = -cam->camera_parameters_.position[0];
      camera_to_world[1] = -cam->camera_parameters_.position[1];
      camera_to_world[2] = -cam->camera_parameters_.position[2];
      ceres::AngleAxisToQuaternion(aa, quat);
      ceres::AngleAxisRotatePoint(aa,camera_to_world,world_to_camera);
      ceres::AngleAxisToRotationMatrix(aa,R);
      ROS_INFO("%s \nPose:\n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf \n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf\n",
	       cam->camera_name_.c_str(),
	       R[0],R[1],R[2],world_to_camera[0],
	       R[3],R[4],R[5],world_to_camera[1],
	       R[6],R[7],R[8],world_to_camera[2],
	       0.0,0.0,0.0,1.0);
      ROS_INFO("Intrinsics:\n fx = %lf fy = %lf\n cx = %lf cy = %lf\n D=[ %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf]",
	       cam->camera_parameters_.focal_length_x,
	       cam->camera_parameters_.focal_length_y,
	       cam->camera_parameters_.center_x,
	       cam->camera_parameters_.center_y,
	       cam->camera_parameters_.distortion_k1,
	       cam->camera_parameters_.distortion_k2,
	       cam->camera_parameters_.distortion_k3,
	       cam->camera_parameters_.distortion_p1,
	       cam->camera_parameters_.distortion_p2);
    }
}
void CeresBlocks::displayMovingCameras()
{
  double R[9];
  double aa[3];
  double camera_to_world[3];
  double world_to_camera[3];
  double quat[4];

  if(moving_cameras_.size() !=0) ROS_INFO("Moving Cameras");
  BOOST_FOREACH(shared_ptr<MovingCamera> mcam, moving_cameras_)
    {
      aa[0] = -mcam->cam->camera_parameters_.angle_axis[0];
      aa[1] = -mcam->cam->camera_parameters_.angle_axis[1];
      aa[2] = -mcam->cam->camera_parameters_.angle_axis[2];
      camera_to_world[0] = -mcam->cam->camera_parameters_.position[0];
      camera_to_world[1] = -mcam->cam->camera_parameters_.position[1];
      camera_to_world[2] = -mcam->cam->camera_parameters_.position[2];
      ceres::AngleAxisToQuaternion(aa, quat);
      ceres::AngleAxisRotatePoint(aa,camera_to_world,world_to_camera);
      ceres::AngleAxisToRotationMatrix(aa,R);
      ROS_INFO("%s \nPose:\n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf \n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf",
	       mcam->cam->camera_name_.c_str(),
	       R[0],R[1],R[2],world_to_camera[0],
	       R[3],R[4],R[5],world_to_camera[1],
	       R[6],R[7],R[8],world_to_camera[2],
	       0.0,0.0,0.0,1.0);
      ROS_INFO("Intrinsics:\n fx = %lf fy = %lf\n cx = %lf cy = %lf\n D=[ %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf]",
	       mcam->cam->camera_parameters_.focal_length_x,
	       mcam->cam->camera_parameters_.focal_length_y,
	       mcam->cam->camera_parameters_.center_x,
	       mcam->cam->camera_parameters_.center_y,
	       mcam->cam->camera_parameters_.distortion_k1,
	       mcam->cam->camera_parameters_.distortion_k2,
	       mcam->cam->camera_parameters_.distortion_k3,
	       mcam->cam->camera_parameters_.distortion_p1,
	       mcam->cam->camera_parameters_.distortion_p2);
    }
}
void CeresBlocks::displayStaticTargets()
{
  double R[9];

  if(static_targets_.size() !=0)   ROS_INFO("Static Targets:");
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
    {
      ceres::AngleAxisToRotationMatrix(targ->pose_.pb_aa,R);
      ROS_INFO("%s \nPose:\n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf \n %d points",
	       targ->target_name_.c_str(),
	       R[0],R[3],R[6],targ->pose_.x,
	       R[1],R[4],R[7],targ->pose_.y,
	       R[2],R[5],R[8],targ->pose_.z,
	       0.0,0.0,0.0,1.0,
	       targ->num_points_);
    }
}
void CeresBlocks::displayMovingTargets()
{
  double R[9];

  if(moving_targets_.size() !=0)   ROS_INFO("Moving Targets:");
  BOOST_FOREACH(shared_ptr<MovingTarget> mtarg, moving_targets_)
    {
      ceres::AngleAxisToRotationMatrix(mtarg->targ_->pose_.pb_aa,R);
      ROS_INFO("%s Pose:\n %6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf \n%6.3lf %6.3lf %6.3lf  %6.3lf\n %6.3lf %6.3lf %6.3lf  %6.3lf\n%d points",
	       mtarg->targ_->target_name_.c_str(),
	       R[0],R[3],R[6],mtarg->targ_->pose_.x,
	       R[1],R[4],R[7],mtarg->targ_->pose_.y,
	       R[2],R[5],R[8],mtarg->targ_->pose_.z,
	       0.0,0.0,0.0,1.0,
	       mtarg->targ_->num_points_);
    }
}
using std::string;
using std::ofstream;
using std::endl;

bool CeresBlocks::writeAllStaticTransforms(string filePath)
{
  std::ofstream outputFile(filePath.c_str(), std::ios::out);// | std::ios::app);
  if (outputFile.is_open())
    {
      ROS_INFO_STREAM("Storing results in: "<<filePath);
    }
  else
    {
      ROS_ERROR_STREAM("Unable to open file:" <<filePath);
      return false;
    }//end if writing to file
  outputFile << "<launch>\n";
  outputFile.close();
  
  bool rtn = true;
  BOOST_FOREACH(shared_ptr<Camera> cam, static_cameras_)
    {
      rtn = cam->transform_interface_->store(filePath);
    }
  BOOST_FOREACH(shared_ptr<MovingCamera> mcam, moving_cameras_)
    {
      rtn = mcam->cam->transform_interface_->store(filePath);
    }
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
    {
      rtn = targ->transform_interface_->store(filePath);
    }
  BOOST_FOREACH(shared_ptr<MovingTarget> mtarg, moving_targets_)
    {
      rtn = mtarg->targ_->transform_interface_->store(filePath);
    }
  
  if(!rtn){
    ROS_ERROR("Couldn't write the static tranform publishers");
  }
  std::ofstream outputFileagain(filePath.c_str(), std::ios::app);
  if (!outputFileagain.is_open())
    {
      ROS_ERROR_STREAM("Unable to re-open file:" <<filePath);
      return false;
    }//end if writing to file
  else{
    outputFileagain << "\n</launch> \n";
    outputFileagain.close();
  }
  return(rtn);
}

void CeresBlocks::pushTransforms()
{
  BOOST_FOREACH(shared_ptr<Camera> cam, static_cameras_)
    {
      cam->pushTransform();
    }
  BOOST_FOREACH(shared_ptr<MovingCamera> mcam, moving_cameras_)
    {
      mcam->cam->pushTransform();
    }
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
    {
      targ->pushTransform();
    }
  BOOST_FOREACH(shared_ptr<MovingTarget> mtarg, moving_targets_)
    {
      mtarg->targ_->pushTransform();
    }

}
void CeresBlocks::pullTransforms()
{
  BOOST_FOREACH(shared_ptr<Camera> cam, static_cameras_)
    {
      cam->pullTransform();
    }
  BOOST_FOREACH(shared_ptr<MovingCamera> mcam, moving_cameras_)
    {
      mcam->cam->pullTransform();
    }
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
    {
      targ->pullTransform();
    }
  BOOST_FOREACH(shared_ptr<MovingTarget> mtarg, moving_targets_)
    {
      mtarg->targ_->pullTransform();
    }
}
void CeresBlocks::setReferenceFrame(std::string ref_frame)
{
  reference_frame_ = ref_frame;
  BOOST_FOREACH(shared_ptr<Camera> cam, static_cameras_)
    {
      cam->setTIReferenceFrame(ref_frame);
    }
  BOOST_FOREACH(shared_ptr<MovingCamera> mcam, moving_cameras_)
    {
      mcam->cam->setTIReferenceFrame(ref_frame);
    }
  BOOST_FOREACH(shared_ptr<Target> targ, static_targets_)
    {
      targ->setTIReferenceFrame(ref_frame);
    }
  BOOST_FOREACH(shared_ptr<MovingTarget> mtarg, moving_targets_)
    {
      mtarg->targ_->setTIReferenceFrame(ref_frame);
    }
}
}// end namespace industrial_extrinsic_cal

