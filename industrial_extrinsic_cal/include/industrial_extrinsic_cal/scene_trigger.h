/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef SCENE_TRIGGER_H_
#define SCENE_TRIGGER_H_

#include <ros/console.h>

namespace industrial_extrinsic_cal
{

  class SceneTrigger
  {
  public:
    /*! \brief Constructor,
     */
    SceneTrigger() {};
    /*! \brief Destructor
     */
    ~SceneTrigger(){};
    /*! \brief Initiates and waits for trigger to finish
     */
    virtual bool waitForTrigger()=0;
  private:
  };
  class ImmediateSceneTrigger: public SceneTrigger
    {
     bool waitForTrigger(){return(true);}; // don't wait
    };
}// end of namespace
#endif
