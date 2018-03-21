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

#ifndef OBSERVATION_SCENE_H_
#define OBSERVATION_SCENE_H_

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/target.h>
#include <ros/console.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace industrial_extrinsic_cal
{
/*! \brief a command to take a set of observations from a group of cameras upon a trigger event */
class ObservationScene
{
public:
  /*! \brief Constructor,
   *   \param Trigger the type of trigger to initiate this observation
   */
  ObservationScene(boost::shared_ptr<Trigger> trigger, int scene_id) : scene_id_(scene_id)
  {
    trigger_ = trigger;
  };

  /*! \brief destructor, clears observation command list*/
  ObservationScene()
  {
    observation_command_list_.clear();
    cameras_in_scene_.clear();
  };

  /*! \brief  Clears all observations from the command */
  void clear();

  /*! \brief Adds and observation of a target by a specific camera in a region of interest
   *  \param observation_command: the camera to make the observation
   *  \param target: the target to be observed
   *  \param roi:    the region of interest in the camera's field of view to look for target
   */
  void addObservationToScene(ObservationCmd observation_command);

  /*!
   * \brief Adds a camera to the scene
   * @param cameras_in_scene
   */
  void addCameraToScene(boost::shared_ptr<Camera> cameras_in_scene);

  /*!
   * \brief will populate the observation_command_list_
   * @param camera the camera for this observation command
   * @param target the target for this observation command
   * @param roi the region of interest for this observation command
   * @param cost_type type of cost function to build with this observation
   */
  void populateObsCmdList(boost::shared_ptr<Camera> camera, boost::shared_ptr<Target> target, Roi roi,
                          Cost_function cost_type);

  /*! \brief gets the id of this scene */
  int get_id()
  {
    return (scene_id_);
  };

  /*! \brief gets the trigger of this scene */
  boost::shared_ptr<Trigger> get_trigger()
  {
    return (trigger_);
  };

  /*!
   * \brief set scene_id for this scene
   * @param sceneId
   */
  void setSceneId(int sceneId)
  {
    scene_id_ = sceneId;
  };

  /*!
   * \brief set the trigger for this scene
   * @param trigger
   */
  void setTrigger(boost::shared_ptr<Trigger> trigger)
  {
    trigger_ = trigger;
  };

  std::vector<ObservationCmd> observation_command_list_;     /*!< list of observations for a scene */
  std::vector<boost::shared_ptr<Camera> > cameras_in_scene_; /*!< list of cameras in this scene */

private:
  boost::shared_ptr<Trigger> trigger_; /*!< event to trigger the observations in this command */
  int scene_id_;                       /*!< unique identifier of this scene */
};
// end of class ObservationScene

}  // end namespace industrial_extrinsic_cal

#endif /* OBSERVATION_SCENE_H_ */
