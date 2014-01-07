/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
#include <industrial_extrinsic_cal/camera_class.h>
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
   *   \param Trigger Trig the type of trigger to initiate this observation
   */
  ObservationScene(Trigger trig, int scene_id) :
      trig_(trig), scene_id_(scene_id)
  {
  }
  ;

  /*! \brief destructor, clears observation command list*/
  ObservationScene()
  {
    observation_command_list_.clear();
    cameras_in_scene_.clear();
  }
  ;

  /*! \brief  Clears all observations from the command */
  void clear();

  /*! \brief Adds and observation of a target by a specific camera in a region of interest
   *  \param observation_command: the camera to make the observation
   *  \param target: the target to be observed
   *  \param roi:    the region of interest in the camera's field of view to look for target
   */
  void addObservationToScene(ObservationCmd observation_command);
  /*! \brief gets the id of this scene */
  int get_id()
  {
    return (scene_id_);
  }
  ;

  /*! \brief gets the trigger of this scene */
  Trigger get_trigger()
  {
    return (trig_);
  }
  ;
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
   * @param trig
   */
  void setTrig(const Trigger& trig)
  {
    trig_ = trig;
  };

  std::vector<ObservationCmd> observation_command_list_; /*!< list of observations for a scene */
  std::vector<boost::shared_ptr<Camera> > cameras_in_scene_; /*!< list of cameras in this scened */

private:
  Trigger trig_; /*!< event to trigger the observations in this command */
  int scene_id_; /*!< unique identifier of this scene */
};
// end of class ObservationScene

}//end namespace industrial_extrinsic_cal

#endif /* OBSERVATION_SCENE_H_ */
