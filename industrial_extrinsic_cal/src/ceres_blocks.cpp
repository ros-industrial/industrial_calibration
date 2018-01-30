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
void showPose(Pose6d pose, std::string message)
{
  tf::Matrix3x3 basis = pose.getBasis();
  double ez_yaw, ey_pitch, ex_roll;
  double qx, qy, qz, qw;
  pose.getEulerZYX(ez_yaw, ey_pitch, ex_roll);
  pose.getQuaternion(qx, qy, qz, qw);
  ROS_INFO("%s =[\n %6.3lf  %6.3lf  %6.3lf  %6.3lf\n  %6.3lf  %6.3lf  %6.3lf  %6.3lf\n  %6.3lf  %6.3lf %6.3lf  "
           "%6.3lf\n  %6.3lf  %6.3lf %6.3lf  %6.3lf];\n rpy= %lf %lf %lf\n quat= %6.3lf %6.3lf %6.3lf %6.3lf ",
           message.c_str(), basis[0][0], basis[0][1], basis[0][2], pose.x, basis[1][0], basis[1][1], basis[1][2],
           pose.y, basis[2][0], basis[2][1], basis[2][2], pose.z, 0.0, 0.0, 0.0, 1.0, ez_yaw, ey_pitch, ex_roll, qx, qy,
           qz, qw);
}

void showPose(P_BLOCK extrinsics, std::string message)
{
  double ax, ay, az, px, py, pz;
  ax = extrinsics[0];
  ay = extrinsics[1];
  az = extrinsics[2];
  px = extrinsics[3];
  py = extrinsics[4];
  pz = extrinsics[5];
  Pose6d pose(px, py, pz, ax, ay, az);
  tf::Matrix3x3 basis = pose.getBasis();
  double ez_yaw, ey_pitch, ex_roll;
  double qx, qy, qz, qw;
  pose.getEulerZYX(ez_yaw, ey_pitch, ex_roll);
  pose.getQuaternion(qx, qy, qz, qw);
  ROS_ERROR("%s =[\n %6.3lf  %6.3lf  %6.3lf  %6.3lf\n  %6.3lf  %6.3lf  %6.3lf  %6.3lf\n  %6.3lf  %6.3lf %6.3lf  "
            "%6.3lf\n  %6.3lf  %6.3lf %6.3lf  %6.3lf];\n rpy= %6.3lf %6.3lf %6.3lf\n quat= %6.3lf  %6.3lf  %6.3lf "
            "%6.3lf ",
            message.c_str(), basis[0][0], basis[0][1], basis[0][2], px, basis[1][0], basis[1][1], basis[1][2], py,
            basis[2][0], basis[2][1], basis[2][2], pz, 0.0, 0.0, 0.0, 1.0, ez_yaw, ey_pitch, ex_roll, qx, qy, qz, qw);
}

void showIntrinsics(P_BLOCK intrinsics, bool with_distortion)
{
  double fx, fy, cx, cy, k1, k2, k3, p1, p2;
  fx = intrinsics[0]; /** focal length x */
  fy = intrinsics[1]; /** focal length y */
  cx = intrinsics[2]; /** central point x */
  cy = intrinsics[3]; /** central point y */
  ROS_INFO("fx = %lf fy=%lf cx=%lf cy=%lf", fx, fy, cx, cy);
  if (with_distortion)
  {
    k1 = intrinsics[4]; /** distortion k1  */
    k2 = intrinsics[5]; /** distortion k2  */
    k3 = intrinsics[6]; /** distortion k3  */
    p1 = intrinsics[7]; /** distortion p1  */
    p2 = intrinsics[8]; /** distortion p2  */
    ROS_INFO("k1 = %lf k2 = %lf k3 = %lf p1 = %lf p2 = %lf", k1, k2, k3, p1, p2);
  }
}

CeresBlocks::CeresBlocks()
{
}
CeresBlocks::~CeresBlocks()
{
  clearCamerasTargets();
}
void CeresBlocks::clearCamerasTargets()
{
  // ROS_INFO_STREAM("Attempting to clear cameras and targets from ceresBlocks");
  static_targets_.clear();
  // ROS_INFO_STREAM("Moving Targets "<<moving_targets_.size());
  moving_targets_.clear();
  // ROS_INFO_STREAM("Static cameras "<<static_cameras_.size());
  static_cameras_.clear();
  // ROS_INFO_STREAM("Moving cameras "<<moving_cameras_.size());
  moving_cameras_.clear();
  // ROS_INFO_STREAM("Cameras and Targets cleared from CeresBlocks");
}
P_BLOCK CeresBlocks::getStaticCameraParameterBlockIntrinsics(string camera_name)
{
  // static cameras should have unique name
  BOOST_FOREACH (shared_ptr<Camera> camera, static_cameras_)
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
  BOOST_FOREACH (shared_ptr<MovingCamera> moving_camera, moving_cameras_)
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
  BOOST_FOREACH (shared_ptr<Camera> camera, static_cameras_)
  {
    if (camera_name == camera->camera_name_)
    {
      P_BLOCK extrinsics = &(camera->camera_parameters_.pb_extrinsics[0]);
      return (extrinsics);
    }
  }
  ROS_ERROR("COULD NOT FIND STATIC CAMERA NAMED %s", camera_name.c_str());
  return (NULL);
}
P_BLOCK CeresBlocks::getMovingCameraParameterBlockExtrinsics(string camera_name, int scene_id)
{
  BOOST_FOREACH (shared_ptr<MovingCamera> camera, moving_cameras_)
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
  BOOST_FOREACH (shared_ptr<Target> target, static_targets_)
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
  BOOST_FOREACH (shared_ptr<Target> target, static_targets_)
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
  BOOST_FOREACH (shared_ptr<MovingTarget> moving_target, moving_targets_)
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
  BOOST_FOREACH (shared_ptr<MovingTarget> moving_target, moving_targets_)
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
  BOOST_FOREACH (shared_ptr<Camera> cam, static_cameras_)
  {
    if (cam->camera_name_ == camera_to_add->camera_name_) return (false);  // camera already exists
  }
  camera_to_add->setTIReferenceFrame(reference_frame_);
  if (camera_to_add->isMoving())
  {
    ROS_ERROR("trying to add a static camera that is moving");
  }
  static_cameras_.push_back(camera_to_add);
  // ROS_INFO_STREAM("Camera added to static_cameras_");
  return (true);
}
bool CeresBlocks::addStaticTarget(shared_ptr<Target> target_to_add)
{
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    if (targ->target_name_ == target_to_add->target_name_)
    {
      return (false);  // target already exists
    }
  }
  target_to_add->setTIReferenceFrame(reference_frame_);
  if (target_to_add->is_moving_)
  {
    ROS_ERROR("trying to add a static target that is moving");
  }
  static_targets_.push_back(target_to_add);

  return (true);
}
bool CeresBlocks::addMovingCamera(shared_ptr<Camera> camera_to_add, int scene_id)
{
  BOOST_FOREACH (shared_ptr<MovingCamera> cam, moving_cameras_)
  {
    if (cam->cam->camera_name_ == camera_to_add->camera_name_ && cam->scene_id == scene_id)
      return (false);  // camera already exists
  }

  // this next line allocates the memory for a moving camera
  shared_ptr<MovingCamera> temp_moving_camera = boost::make_shared<MovingCamera>();

  // this next line allocates the memory for the actual camera
  shared_ptr<Camera> temp_camera =
      boost::make_shared<Camera>(camera_to_add->camera_name_, camera_to_add->camera_parameters_, true);

  // set things not done by constructor using values from camera_to_add
  temp_camera->setTransformInterface(camera_to_add->getTransformInterface());
  temp_camera->setTIReferenceFrame(reference_frame_);
  temp_camera->camera_observer_ = camera_to_add->camera_observer_;
  temp_camera->intermediate_frame_ = camera_to_add->intermediate_frame_;

  temp_moving_camera->cam = temp_camera;
  temp_moving_camera->scene_id = scene_id;
  moving_cameras_.push_back(temp_moving_camera);
  return (true);
}
bool CeresBlocks::addMovingTarget(shared_ptr<Target> target_to_add, int scene_id)
{
  BOOST_FOREACH (shared_ptr<MovingTarget> targ, moving_targets_)
  {
    if (targ->targ_->target_name_ == target_to_add->target_name_ && targ->scene_id_ == scene_id)
    {
      targ->targ_->pose_ = target_to_add->pose_;  // update pose at least to account for intialization issue
      return (false);                             // target already exists
    }
  }

  // deep copy of target into moving target TODO test to see if using * or contents of notation can avoid all this
  // direct copy
  shared_ptr<MovingTarget> temp_moving_target = boost::make_shared<MovingTarget>();
  temp_moving_target->targ_ = boost::make_shared<Target>();
  temp_moving_target->targ_->pose_ = target_to_add->pose_;
  temp_moving_target->targ_->num_points_ = target_to_add->num_points_;
  temp_moving_target->targ_->pts_ = target_to_add->pts_;
  temp_moving_target->targ_->is_moving_ = target_to_add->is_moving_;
  temp_moving_target->targ_->target_name_ = target_to_add->target_name_;
  temp_moving_target->targ_->target_frame_ = target_to_add->target_frame_;
  temp_moving_target->targ_->target_type_ = target_to_add->target_type_;
  temp_moving_target->scene_id_ = scene_id;
  if (temp_moving_target->targ_->target_type_ == 0)
  {
    temp_moving_target->targ_->checker_board_parameters_ = target_to_add->checker_board_parameters_;
  }
  else if (temp_moving_target->targ_->target_type_ == 1 || temp_moving_target->targ_->target_type_ == 2)
  {
    temp_moving_target->targ_->circle_grid_parameters_ = target_to_add->circle_grid_parameters_;
  }
  else
  {
    temp_moving_target->targ_->ar_target_parameters_ = target_to_add->ar_target_parameters_;
  }
  temp_moving_target->targ_->setTransformInterface(target_to_add->getTransformInterface());
  boost::shared_ptr<TransformInterface> the_interface = target_to_add->getTransformInterface();
  if (the_interface->isRefFrameInitialized())
  {
    std::string ref_frame;
    ref_frame = the_interface->getReferenceFrame();
    temp_moving_target->targ_->setTIReferenceFrame(ref_frame);
  }
  // add the target to the list
  moving_targets_.push_back(temp_moving_target);
  return (true);
}

const boost::shared_ptr<Camera> CeresBlocks::getCameraByName(const std::string& camera_name)
{
  boost::shared_ptr<Camera> cam = boost::make_shared<Camera>();
  // ROS_INFO_STREAM("Found "<<static_cameras_.size() <<" static cameras");
  for (int i = 0; i < static_cameras_.size(); i++)
  {
    if (static_cameras_.at(i)->camera_name_ == camera_name)
    {
      cam = static_cameras_.at(i);
      ROS_DEBUG_STREAM("Found static camera with name: " << static_cameras_.at(i)->camera_name_);
    }
  }
  // ROS_INFO_STREAM("Found "<<moving_cameras_.size() <<" moving cameras");
  for (int i = 0; i < moving_cameras_.size(); i++)
  {
    if (moving_cameras_.at(i)->cam->camera_name_ == camera_name)
    {
      cam = moving_cameras_.at(i)->cam;
      ROS_DEBUG_STREAM("Found moving camera with name: " << camera_name);
    }
  }
  if (!cam)
  {
    ROS_ERROR("getCameraByName Failed for %s", camera_name.c_str());
  }
  return cam;
  // return true;
}

const boost::shared_ptr<Target> CeresBlocks::getTargetByName(const std::string& target_name, int scene_id)
{
  boost::shared_ptr<Target> target = boost::make_shared<Target>();
  bool found = false;
  // ROS_INFO_STREAM("Found "<<static_cameras_.size() <<" static cameras");
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    if (targ->target_name_ == target_name)
    {
      target = targ;
      found = true;
      ROS_DEBUG_STREAM("Found static target with name: " << target_name);
    }
  }
  // ROS_INFO_STREAM("Found "<<moving_cameras_.size() <<" static cameras");
  BOOST_FOREACH (shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    if (mtarg->targ_->target_name_ == target_name && mtarg->scene_id_ == scene_id)
    {
      found = true;
      target = mtarg->targ_;
      ROS_DEBUG_STREAM("Found moving target with name: " << target_name);
    }
  }
  if (!found)
  {
    ROS_ERROR("getStaticTargetByName Failed for %s", target_name.c_str());
  }
  return target;
}

void CeresBlocks::displayStaticCameras()
{
  if (static_cameras_.size() != 0) ROS_INFO("Static Cameras");
  BOOST_FOREACH (shared_ptr<Camera> cam, static_cameras_)
  {
    Pose6d pose(cam->camera_parameters_.position[0], cam->camera_parameters_.position[1],
                cam->camera_parameters_.position[2], cam->camera_parameters_.angle_axis[0],
                cam->camera_parameters_.angle_axis[1], cam->camera_parameters_.angle_axis[2]);
    Pose6d ipose = pose.getInverse();
    showPose(ipose, cam->camera_name_);
    showIntrinsics(cam->camera_parameters_.pb_intrinsics, true);
  }
}
void CeresBlocks::displayMovingCameras()
{
  tf::Matrix3x3 R;
  double aa[3];
  double camera_to_world[3];
  double world_to_camera[3];
  double quat[4];

  if (moving_cameras_.size() != 0) ROS_INFO("Moving Cameras");
  BOOST_FOREACH (shared_ptr<MovingCamera> mcam, moving_cameras_)
  {
    if (mcam->scene_id == 0)
    {
      Pose6d pose(mcam->cam->camera_parameters_.position[0], mcam->cam->camera_parameters_.position[1],
                  mcam->cam->camera_parameters_.position[2], mcam->cam->camera_parameters_.angle_axis[0],
                  mcam->cam->camera_parameters_.angle_axis[1], mcam->cam->camera_parameters_.angle_axis[2]);
      Pose6d ipose = pose.getInverse();
      showPose(ipose, mcam->cam->camera_name_);
      P_BLOCK intrinsics = getMovingCameraParameterBlockIntrinsics(mcam->cam->camera_name_);
      showIntrinsics(intrinsics, true);
    }
  }
}
void CeresBlocks::displayStaticTargets()
{
  double R[9];

  if (static_targets_.size() != 0) ROS_INFO("Static Targets:");
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    showPose(targ->pose_, targ->target_name_);
  }
}
void CeresBlocks::displayMovingTargets()
{
  double R[9];

  if (moving_targets_.size() != 0) ROS_INFO("Moving Targets:");
  BOOST_FOREACH (shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    if (mtarg->scene_id_ == 0)
    {  // only show first pose, not all
      showPose(mtarg->targ_->pose_, mtarg->targ_->target_name_);
    }
  }
}
using std::string;
using std::ofstream;
using std::endl;

bool CeresBlocks::writeAllStaticTransforms(string filePath)
{
  std::ofstream outputFile(filePath.c_str(), std::ios::out);  // | std::ios::app);
  if (outputFile.is_open())
  {
    ROS_INFO_STREAM("Storing results in: " << filePath);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open file:" << filePath);
    return false;
  }  // end if writing to file
  outputFile << "<launch>\n";
  outputFile << "# this file might be empty\n";
  outputFile << "# It might contain static transform publishers that were updated through calibration\n";
  outputFile << "# Often, transform interfaces use some other mechanism to update themselves\n";
  outputFile.close();

  bool rtn = true;
  BOOST_FOREACH (shared_ptr<Camera> cam, static_cameras_)
  {
    rtn = cam->transform_interface_->store(filePath);
  }
  BOOST_FOREACH (shared_ptr<MovingCamera> mcam, moving_cameras_)
  {
    rtn = mcam->cam->transform_interface_->store(filePath);
  }
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    rtn = targ->transform_interface_->store(filePath);
  }
  BOOST_FOREACH (shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    rtn = mtarg->targ_->transform_interface_->store(filePath);
  }

  if (!rtn)
  {
    ROS_ERROR("Couldn't write the static tranform publishers");
  }
  std::ofstream outputFileagain(filePath.c_str(), std::ios::app);
  if (!outputFileagain.is_open())
  {
    ROS_ERROR_STREAM("Unable to re-open file:" << filePath);
    return false;
  }  // end if writing to file
  else
  {
    outputFileagain << "\n</launch> \n";
    outputFileagain.close();
  }
  return (rtn);
}

void CeresBlocks::pushTransforms()
{
  BOOST_FOREACH (shared_ptr<Camera> cam, static_cameras_)
  {
    cam->pushTransform();
  }
  BOOST_FOREACH (shared_ptr<MovingCamera> mcam, moving_cameras_)
  {
    if (mcam->scene_id == 0)
    {  // only push a moving camera's transform once, which is always scene 0
      Pose6d pose;
      pose.setAngleAxis(mcam->cam->camera_parameters_.angle_axis[0], mcam->cam->camera_parameters_.angle_axis[1],
                        mcam->cam->camera_parameters_.angle_axis[2]);
      pose.setOrigin(mcam->cam->camera_parameters_.position[0], mcam->cam->camera_parameters_.position[1],
                     mcam->cam->camera_parameters_.position[2]);
      mcam->cam->pushTransform();
    }
  }
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    targ->pushTransform();
  }
  BOOST_FOREACH (shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    if (mtarg->scene_id_ == 0)
    {  // only push a moving target once which is for the scene 0
      mtarg->targ_->pushTransform();
    }
  }
}
void CeresBlocks::pullTransforms(int scene_id)
{
  BOOST_FOREACH (shared_ptr<Camera> cam, static_cameras_)
  {
    cam->pullTransform();
  }
  BOOST_FOREACH (shared_ptr<MovingCamera> mcam, moving_cameras_)
  {
    if (mcam->scene_id == scene_id)
    {  // only pull transforms for cameras in current scene
      mcam->cam->pullTransform();
    }
  }
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    targ->pullTransform();
  }
  BOOST_FOREACH (shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    if (mtarg->scene_id_ == scene_id)
    {  // only pull transforms for targets in current scene
      mtarg->targ_->pullTransform();
    }
  }
}
void CeresBlocks::setReferenceFrame(std::string ref_frame)
{
  reference_frame_ = ref_frame;
  BOOST_FOREACH (shared_ptr<Camera> cam, static_cameras_)
  {
    cam->setTIReferenceFrame(ref_frame);
  }
  BOOST_FOREACH (shared_ptr<MovingCamera> mcam, moving_cameras_)
  {
    mcam->cam->setTIReferenceFrame(ref_frame);
  }
  BOOST_FOREACH (shared_ptr<Target> targ, static_targets_)
  {
    targ->setTIReferenceFrame(ref_frame);
  }
  BOOST_FOREACH (shared_ptr<MovingTarget> mtarg, moving_targets_)
  {
    mtarg->targ_->setTIReferenceFrame(ref_frame);
  }
}
}  // end namespace industrial_extrinsic_cal
