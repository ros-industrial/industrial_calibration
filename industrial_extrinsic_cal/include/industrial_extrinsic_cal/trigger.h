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

#ifndef TRIGGER_H_
#define TRIGGER_H_

namespace industrial_extrinsic_cal
{
/*! @brief what kind of trigger initiates the collection of data for this scene */
class Trigger
{ /** Trigger */
public:
  /*! \brief Constructor,
   */
  Trigger(){};

  /*! \brief Destructor
   */
  virtual ~Trigger(){};

  /*! \brief Initiates and waits for trigger to finish
   */
  virtual bool waitForTrigger() = 0;
};

class NoWaitTrigger : public Trigger
{
  bool waitForTrigger()
  {
    return (true);
  };  // don't wait
};

}  // end of namespace

#endif
