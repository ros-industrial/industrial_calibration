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

#ifndef ROS_TRANSFORM_INTERFACE_H_
#define ROS_TRANSFORM_INTERFACE_H_

#include <stdio.h>
#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/basic_types.h> // for Pose6d

namespace industrial_extrinsic_cal
{

  /** @brief this object is intened to be used for targets, not cameras
   *            It simply listens to a pose from ref to transform frame, this must be set in a urdf
   *            push does nothing
   *            pull listens to tf for desired transform
   *            store does nothing
   */
  class ROSListenerTransInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param image_topic name of published image topic
     */
    ROSListenerTransInterface(const std::string transform_frame);

    /**
     * @brief Default destructor
     */
    ~ROSListenerTransInterface()  {  } ;

    /** @brief  this is a listener interface, so this does nothing */
    bool pushTransform(Pose6d & Pose){ return(true);};

    /** @brief get the transform from tf */
    Pose6d pullTransform();

    /** @brief this is a listener interface, does nothing */
    bool store(std::string filePath) { return(true);};

  private:
    Pose6d pose_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_;
  };

  /** @brief this object is intened to be used for cameras not targets
   *            It simply listens to a pose from camera's optical frame to reference frame, this must be set in a urdf
   *            This is the inverse of the transform from world to camera's optical frame (opposite of the target transform)
   *            push does nothing
   *            pull listens to tf for desired transform
   *            store does nothing
   */
  class ROSCameraListenerTransInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param image_topic name of published image topic
     */
    ROSCameraListenerTransInterface(const std::string transform_frame);

    /**
     * @brief Default destructor
     */
    ~ROSCameraListenerTransInterface()  {  } ;

    /** @brief  the object is a listener, so this does nothing*/
    bool pushTransform(Pose6d & Pose){ return(true);};

    /** @brief this returns the transform from the optical frame to the reference frame as returned by tf */
    Pose6d pullTransform();

    /** @brief as a listener interface, this does nothing because transform defined by urdf*/
    bool store(std::string filePath) {return(true);};

  private:
    Pose6d pose_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_;
  };

  /** @brief this is expected to be used by a camera who's position is defined by the urdf
   *              For example the camera might be held by a robot. 
   *              The pose kept internally is from the camera's optical frame to the reference frame
   *            push does nothing
   *            pull listens to tf for desired transform
   *            store does nothing
   */
  class ROSCameraHousingListenerTInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param optical_frame The name of the optical frame defined in the urdf
     * @param optical_frame The name of camera housing frame defined in the urdf
     */
    ROSCameraHousingListenerTInterface(const std::string optical_frame,const std::string housing_frame);

    /**
     * @brief Default destructor
     */
    ~ROSCameraHousingListenerTInterface()  {  } ;

    /** @brief  as a listener interface, this does nothing. Transform is defined by the urdf */
    bool pushTransform(Pose6d & Pose){return(true);};

    /** @brief get the transform from the hardware or display */
    Pose6d pullTransform();

    /** @brief as a listener interface, this does nothing. Transform is defined by urdf */
    bool store(std::string filePath){ return(true);};

  private:
    std::string housing_frame_; /**< housing frame name note, this is not used, but kept be symetry with broadcaster param list */
    Pose6d pose_; /**< pose associated with the transform from reference frame to housing frame */
    tf::TransformListener tf_listener_; /**< listener used to get the transform/pose_ from tf */
    tf::StampedTransform transform_;/**< listener returns this which we convert to pose_ */
  };

  /** @brief This transform interface is used when the pose determined through calibration
      /* The urdf should not define this transform, otherwise there will be a conflict
      /* the pose from the yaml file is broadcast immediately 
      /* Once calibrated, the pose may be pushed, then the updated pose will be observed by tf
  */
  class ROSBroadcastTransInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param image_topic name of published image topic
     */
    ROSBroadcastTransInterface(const std::string transform_frame, Pose6d pose);

    /**
     * @brief Default destructor
     */
    ~ROSBroadcastTransInterface()  {  } ;

    /** @brief  updates the pose being broadcast*/
    bool pushTransform(Pose6d & pose);

    /** @brief this is a broadcaster, this should return the value of the construtor, or the last value used in pullTransform(pose) */
    Pose6d pullTransform() {return(pose_);};

    /** @brief appends data as a static transform publisher to the file */
    bool store(std::string filePath);

    /** @brief a sets ref_frame_, but also starts the timer for broadcasting the transform */
    void setReferenceFrame(std::string ref_frame);

    /** @brief a timer callback that continuously broadcast the current pose as a tf */
    void timerCallback(const ros::TimerEvent & timer_event);
   
  private:
    Pose6d pose_; /**< pose associated with the transform */
    ros::Timer timer_; /**< need a timer to initiate broadcast of transform */
    tf::StampedTransform transform_; /**< the broadcaster needs this which we get values from pose_ */
    tf::TransformBroadcaster tf_broadcaster_; /**< the broadcaster to tf */
    bool ref_frame_defined_; /**< the broadcaster can't start until the reference frame is defined, this is set then */
  };

  /** @brief This transform interface is used when the camera pose  is determined through calibration
      /* The urdf should not define this transform, otherwise there will be a conflict
      /* the pose in the camera yaml file is broadcast imediately 
      /* Once calibrated, the pose may be pushed to tf
  */

  class ROSCameraBroadcastTransInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param image_topic name of published image topic
     */
    ROSCameraBroadcastTransInterface(const std::string transform_frame, Pose6d pose);

    /**
     * @brief Default destructor
     */
    ~ROSCameraBroadcastTransInterface()  {  } ;

    /** @brief  updates the pose being broadcast*/
    bool pushTransform(Pose6d & pose);

    /** @brief this is a broadcaster, this should return the value of the construtor, or the last value used in pullTransform(pose) */
    Pose6d pullTransform() {return(pose_);};

    /** @brief appends data as a static transform publisher to the file */
    bool store(std::string filePath);

    /** @brief a sets ref_frame_, but also starts the timer for broadcasting the transform */
    void setReferenceFrame(std::string ref_frame);

    /** @brief a timer callback that continuously broadcast the current pose as a tf */
    void timerCallback(const ros::TimerEvent & timer_event);
   
  private:
    Pose6d pose_; /**< pose associated with the transform */
    ros::Timer timer_; /**< need a timer to initiate broadcast of transform */
    tf::StampedTransform transform_; /**< the broadcaster needs this which we get values from pose_ */
    tf::TransformBroadcaster tf_broadcaster_; /**< the broadcaster to tf */
    bool ref_frame_defined_; /**< the broadcaster can't start until the reference frame is defined, this is set then */
  };


  /** @brief This transform interface is used when the camera pose  is determined through calibration
      /* The urdf should not define this transform, otherwise there will be a conflict
      /* the pose in the camera yaml file is broadcast imediately 
      /* Once calibrated, the pose may be pushed to tf
  */
  class ROSCameraHousingBroadcastTInterface : public TransformInterface
  {
  public:

    /**
     * @brief constructor
     * @param image_topic name of published image topic
     */
    ROSCameraHousingBroadcastTInterface(const std::string transform_frame, Pose6d pose);

    /**
     * @brief Default destructor
     */
    ~ROSCameraHousingBroadcastTInterface()  {  } ;

    /** @brief  updates the pose being broadcast*/
    bool pushTransform(Pose6d & Pose);

    /** @brief returns the pose used in construction, or the one most recently pushed */
    Pose6d pullTransform(){ return(pose_);};

    /** @brief appends pose as a static transform publisher to the file */
    bool store(std::string filePath);

    /** @brief a timer callback that continuously broadcast the current pose as a tf */
    void timerCallback(const ros::TimerEvent & timer_event);
   
    /** @brief initialized the reference frame, and starts broadcasting with current value of pose_ */
    void  setReferenceFrame(std::string ref_frame);
  private:
    Pose6d pose_; /**< pose associated with the transform */
    ros::Timer timer_; /**< need a timer to initiate broadcast of transform */
    tf::StampedTransform transform_; /**< the broadcaster needs this which we get values from pose_ */
    tf::TransformBroadcaster tf_broadcaster_; /**< the broadcaster to tf */
    tf::TransformListener tf_listener_; // need this to get the tranform from housing to optical frame from tf
    bool ref_frame_defined_; /**< the broadcaster can't start until the reference frame is defined, this is set then */
    std::string housing_frame_; /**< frame name for the housing */
  };
} //end industrial_extrinsic_cal namespace
#endif /* ROS_CAMERA_OBSERVER_H_ */
