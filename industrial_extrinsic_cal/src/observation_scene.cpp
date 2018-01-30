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

#include <industrial_extrinsic_cal/observation_scene.h>
#include <boost/shared_ptr.hpp>

using boost::shared_ptr;

namespace industrial_extrinsic_cal
{
void ObservationScene::addObservationToScene(ObservationCmd new_obs_cmd)
{
  // this next block of code maintains a list of the cameras in a scene
  bool camera_already_in_scene = false;
  BOOST_FOREACH (ObservationCmd command, observation_command_list_)
  {
    BOOST_FOREACH (shared_ptr<Camera> camera, cameras_in_scene_)
    {
      if (camera->camera_name_ == new_obs_cmd.camera->camera_name_)
      {
        camera_already_in_scene = true;
      }
    }
  }
  if (!camera_already_in_scene)
  {
    cameras_in_scene_.push_back(new_obs_cmd.camera);
  }
  // end of code block to maintain list of cameras in scene

  // add observation
  observation_command_list_.push_back(new_obs_cmd);
}

void ObservationScene::addCameraToScene(boost::shared_ptr<Camera> camera_in_scene)
{
  cameras_in_scene_.push_back(camera_in_scene);
  ROS_DEBUG_STREAM("Added camera " << camera_in_scene->camera_name_
                                   << " to cameras_in_scene list of size: " << cameras_in_scene_.size());
}

void ObservationScene::populateObsCmdList(boost::shared_ptr<Camera> camera, boost::shared_ptr<Target> target, Roi roi,
                                          Cost_function cost_type)
{
  ObservationCmd current_obs_cmd;
  current_obs_cmd.camera = camera;
  current_obs_cmd.target = target;
  current_obs_cmd.roi = roi;
  current_obs_cmd.cost_type = cost_type;
  observation_command_list_.push_back(current_obs_cmd);
}

}  // end namespace industrial_extrinsic_cal
