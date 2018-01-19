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
#include <industrial_extrinsic_cal/basic_types.h>  // for Pose6d
#include <industrial_extrinsic_cal/get_mutable_joint_states.h>
#include <industrial_extrinsic_cal/set_mutable_joint_states.h>
#include <industrial_extrinsic_cal/store_mutable_joint_states.h>
#include <boost/make_shared.hpp>

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
  ROSListenerTransInterface(const std::string& transform_frame);

  /**
   * @brief Default destructor
   */
  ~ROSListenerTransInterface(){};

  /** @brief  this is a listener interface, so this does nothing */
  bool pushTransform(Pose6d& Pose)
  {
    return (true);
  };

  /** @brief get the transform from tf */
  Pose6d pullTransform();

  /** @brief this is a listener interface, does nothing */
  bool store(std::string& filePath)
  {
    return (true);
  };

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame)
  {
    ref_frame_ = ref_frame;
    ref_frame_initialized_ = true;
  }

private:
  Pose6d pose_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform_;
};

/** @brief this object is intened to be used for cameras not targets
 *            It simply listens to a pose from camera's optical frame to reference frame, this must be set in a urdf
 *            This is the inverse of the transform from world to camera's optical frame (opposite of the target
 * transform)
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
  ROSCameraListenerTransInterface(const std::string& transform_frame);

  /**
   * @brief Default destructor
   */
  ~ROSCameraListenerTransInterface(){};

  /** @brief  the object is a listener, so this does nothing*/
  bool pushTransform(Pose6d& Pose)
  {
    return (true);
  };

  /** @brief this returns the transform from the optical frame to the reference frame as returned by tf */
  Pose6d pullTransform();

  /** @brief as a listener interface, this does nothing because transform defined by urdf*/
  bool store(std::string& filePath)
  {
    return (true);
  };

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame)
  {
    ref_frame_ = ref_frame;
    ref_frame_initialized_ = true;
  }

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
   * @param housing_frame The name of camera housing frame defined in the urdf
   */
  ROSCameraHousingListenerTInterface(const std::string& optical_frame, const std::string& housing_frame);

  /**
   * @brief Default destructor
   */
  ~ROSCameraHousingListenerTInterface(){};

  /** @brief  as a listener interface, this does nothing. Transform is defined by the urdf */
  bool pushTransform(Pose6d& Pose)
  {
    return (true);
  };

  /** @brief get the transform from the hardware or display */
  Pose6d pullTransform();

  /** @brief as a listener interface, this does nothing. Transform is defined by urdf */
  bool store(std::string& filePath)
  {
    return (true);
  };

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame)
  {
    ref_frame_ = ref_frame;
    ref_frame_initialized_ = true;
  }

private:
  std::string
      housing_frame_; /**< housing frame name note, this is not used, but kept be symetry with broadcaster param list */
  Pose6d pose_;       /**< pose associated with the transform from reference frame to housing frame */
  tf::TransformListener tf_listener_; /**< listener used to get the transform/pose_ from tf */
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
   * @param transform_frame the frame associated with this transform
   */
  ROSBroadcastTransInterface(const std::string& transform_frame);

  /**
   * @brief Default destructor
   */
  ~ROSBroadcastTransInterface(){};

  /** @brief  updates the pose being broadcast*/
  bool pushTransform(Pose6d& pose);

  /** @brief this is a broadcaster, this should return the value of the construtor, or the last value used in
   * pullTransform(pose) */
  Pose6d pullTransform()
  {
    return (pose_);
  };

  /** @brief appends data as a static transform publisher to the file */
  bool store(std::string& filePath);

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame);

  /** @brief a timer callback that continuously broadcast the current pose as a tf */
  void timerCallback(const ros::TimerEvent& timer_event);

private:
  Pose6d pose_;                             /**< pose associated with the transform */
  ros::Timer timer_;                        /**< need a timer to initiate broadcast of transform */
  tf::StampedTransform transform_;          /**< the broadcaster needs this which we get values from pose_ */
  tf::TransformBroadcaster tf_broadcaster_; /**< the broadcaster to tf */
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
   * @param transform_frame the frame associated with this transform
   */
  ROSCameraBroadcastTransInterface(const std::string& transform_frame);

  /**
   * @brief Default destructor
   */
  ~ROSCameraBroadcastTransInterface(){};

  /** @brief  updates the pose being broadcast*/
  bool pushTransform(Pose6d& pose);

  /** @brief this is a broadcaster, this should return the value of the construtor, or the last value used in
   * pullTransform(pose) */
  Pose6d pullTransform()
  {
    return (pose_);
  };

  /** @brief appends data as a static transform publisher to the file */
  bool store(std::string& filePath);

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame);

  /** @brief a timer callback that continuously broadcast the current pose as a tf */
  void timerCallback(const ros::TimerEvent& timer_event);

private:
  Pose6d pose_;                             /**< pose associated with the transform */
  ros::Timer timer_;                        /**< need a timer to initiate broadcast of transform */
  tf::StampedTransform transform_;          /**< the broadcaster needs this which we get values from pose_ */
  tf::TransformBroadcaster tf_broadcaster_; /**< the broadcaster to tf */
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
   * @param transform_frame the frame associated with this transform
   * @param housing_frame the camera's housing frame
   * @param mounting_frame the camera's mounting frame
   */
  ROSCameraHousingBroadcastTInterface(const std::string& transform_frame, const std::string& housing_frame,
                                      const std::string& mounting_frame, const Pose6d& pose);

  /**
   * @brief Default destructor
   */
  ~ROSCameraHousingBroadcastTInterface(){};

  /** @brief  updates the pose being broadcast*/
  bool pushTransform(Pose6d& Pose);

  /** @brief returns the pose used in construction, or the one most recently pushed */
  Pose6d pullTransform();

  /** @brief appends pose as a static transform publisher to the file */
  bool store(std::string& filePath);

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame);

  /** @brief a timer callback that continuously broadcast the current pose as a tf */
  void timerCallback(const ros::TimerEvent& timer_event);

private:
  Pose6d pose_;                             /**< pose associated with the transform */
  ros::Timer timer_;                        /**< need a timer to initiate broadcast of transform */
  tf::StampedTransform transform_;          /**< the broadcaster needs this which we get values from pose_ */
  tf::TransformBroadcaster tf_broadcaster_; /**< the broadcaster to tf */
  tf::TransformListener tf_listener_;       // need this to get the tranform from housing to optical frame from tf
  std::string housing_frame_;               /**< frame name for the housing */
  std::string mounting_frame_;              /**< mounting frame name */
};

/** @brief this is expected to be used by a camera who's position is defined by the urdf using the calibration xacro
 * macro
 *             The macro takes a child and a parent link, and defines the following joints
 *             {child}_x_joint:
 *             {child}_y_joint:
 *             {child}_z_joint:
 *             {child}_yaw_joint:
 *             {child}_pitch_joint:
 *             {child}_roll_joint:
 *             it is expected that the launch file for the workcell will include a mutable_joint_state_publisher
 *             and that these joints are defined in the file whose name is held in the  mutableJointStateYamlFile
 * parameter
 *             push updates the joint states by calling the set_mutable_joint_states service
 *             pull listens to the joint states by calling the get_mutable_joint_states service, and computes the
 * transform
 *             store updates the yaml file by calling the store_mutable_joint_states service
 */
class ROSCameraHousingCalTInterface : public TransformInterface
{
public:
  /**
   * @brief constructor
   * @param transform_frame The name of camera's optical frame
   * @param housing_frame The name of camera housing's frame
   * @param mounting_frame The name of frame on which the camera housing is mounted
   */
  ROSCameraHousingCalTInterface(const std::string& transform_frame, /* optical frame */
                                const std::string& housing_frame, const std::string& mounting_frame);

  /**
   * @brief Default destructor
   */
  ~ROSCameraHousingCalTInterface(){};

  /** @brief  uses the pose to compute the new joint values, and sends them to the mutable joint state publisher
   * @param Pose the pose is the transform from the optical frame to the reference frame
   * This function assumes this Pose is composed of three frames
   * one from optical frame to housing frame
   * one from housing to mounting frame
   * one from mounting to reference
   * the middle one is the one which is decomposed into the 6DOF joint values,
   */
  bool pushTransform(Pose6d& Pose);

  /** @brief get the transform from the mutable transform publisher, and compute the optical to reference frame pose*/
  Pose6d pullTransform();

  /** @brief as a listener interface, tells the mutable transform publisher to store its current values in its yaml file
   */
  bool store(std::string& filePath);

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame);

  /** @brief used to get the pose of the mounting frame in ref_frame coordinates
   *   @return returns the pose from ref_frame to mounting frame in the case where necessary, identity otherwise
   */
  Pose6d getIntermediateFrame();

private:
  ros::NodeHandle* nh_;               /**< the standard node handle for ros*/
  std::string housing_frame_;         /**< housing frame name */
  std::string mounting_frame_;        /**< mounting frame name */
  Pose6d pose_;                       /**< pose associated with the transform from reference frame to housing frame */
  tf::TransformListener tf_listener_; /**< listener used to get the transform/pose_ from tf */
  ros::ServiceClient get_client_;     /**< a client for calling the service to get the joint values associated with the
                                         transform */
  ros::ServiceClient set_client_;     /**< a client for calling the service to set the joint values associated with the
                                         transform */
  ros::ServiceClient
      store_client_; /**< a client for calling the service to store the joint values associated with the transform */
  std::vector<std::string> joint_names_;                                    /**< names of joints  */
  std::vector<double> joint_values_;                                        /**< values of joints  */
  industrial_extrinsic_cal::get_mutable_joint_states::Request get_request_; /**< request when transform is part of a
                                                                               mutable set */
  industrial_extrinsic_cal::get_mutable_joint_states::Response
      get_response_; /**< response when transform is part of a mutable set */
  industrial_extrinsic_cal::set_mutable_joint_states::Request set_request_; /**< request when transform is part of a
                                                                               mutable set */
  industrial_extrinsic_cal::set_mutable_joint_states::Response
      set_response_; /**< response when transform is part of a mutable set */
  industrial_extrinsic_cal::store_mutable_joint_states::Request store_request_; /**< request to store when  part of a
                                                                                   mutable set */
  industrial_extrinsic_cal::store_mutable_joint_states::Response
      store_response_; /**< response to store when  part of a mutable set */
};

/** @brief this is expected to be used for calibrating a target
 *             The macro takes a child and a parent link, and defines the following joints
 *             {child}_x_joint:
 *             {child}_y_joint:
 *             {child}_z_joint:
 *             {child}_yaw_joint:
 *             {child}_pitch_joint:
 *             {child}_roll_joint:
 *             it is expected that the launch file for the workcell will include a mutable_joint_state_publisher
 *             and that these joints are defined in the file whose name is held in the  mutableJointStateYamlFile
 * parameter
 *             push updates the joint states by calling the set_mutable_joint_states service
 *             pull listens to the joint states by calling the get_mutable_joint_states service, and computes the
 * transform
 *             store updates the yaml file by calling the store_mutable_joint_states service
 */
class ROSSimpleCalTInterface : public TransformInterface
{
public:
  /**
   * @brief constructor
   * @param transform_frame The name of frame being calibrated
   * @param parent_frame The name of camera housing's frame
   */
  ROSSimpleCalTInterface(const std::string& transform_frame, const std::string& parent_frame);

  /**
   * @brief Default destructor
   */
  ~ROSSimpleCalTInterface(){};

  /** @brief  uses the pose to compute the new joint values, and sends them to the mutable joint state publisher
   * @param Pose the pose is the transform from the parent frame to the transform
   */
  bool pushTransform(Pose6d& Pose);

  /** @brief get the transform from the mutable transform publisher, and compute the optical to reference frame pose*/
  Pose6d pullTransform();

  /** @brief as a listener interface, tells the mutable transform publisher to store its current values in its yaml file
   */
  bool store(std::string& filePath);

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame);

private:
  ros::NodeHandle* nh_;
  std::string transform_frame_;   /**< transform frame name */
  std::string parent_frame_;      /**< parent's frame name */
  Pose6d pose_;                   /**< pose associated with the transform from reference frame to housing frame */
  ros::ServiceClient get_client_; /**< a client for calling the service to get the joint values associated with the
                                     transform */
  ros::ServiceClient set_client_; /**< a client for calling the service to set the joint values associated with the
                                     transform */
  ros::ServiceClient
      store_client_; /**< a client for calling the service to store the joint values associated with the transform */
  std::vector<std::string> joint_names_; /**< names of joints  */
  std::vector<double> joint_values_;     /**< values of joints  */
  industrial_extrinsic_cal::get_mutable_joint_states::Request get_request_;
  industrial_extrinsic_cal::get_mutable_joint_states::Response get_response_;
  industrial_extrinsic_cal::set_mutable_joint_states::Request set_request_;
  industrial_extrinsic_cal::set_mutable_joint_states::Response set_response_;
  industrial_extrinsic_cal::store_mutable_joint_states::Request store_request_;
  industrial_extrinsic_cal::store_mutable_joint_states::Response store_response_;
};

/** @brief this is expected to be used for calibrating a camera
 *             The macro takes a child and a parent link, and defines the following joints
 *             {child}_x_joint:
 *             {child}_y_joint:
 *             {child}_z_joint:
 *             {child}_yaw_joint:
 *             {child}_pitch_joint:
 *             {child}_roll_joint:
 *             it is expected that the launch file for the workcell will include a mutable_joint_state_publisher
 *             and that these joints are defined in the file whose name is held in the  mutableJointStateYamlFile
 * parameter
 *             push updates the joint states by calling the set_mutable_joint_states service
 *             pull listens to the joint states by calling the get_mutable_joint_states service, and computes the
 * transform
 *             store updates the yaml file by calling the store_mutable_joint_states service
 */
class ROSSimpleCameraCalTInterface : public TransformInterface
{
public:
  /**
   * @brief constructor
   * @param transform_frame The name of frame being calibrated
   * @param parent_frame The name of camera housing's frame
   */
  ROSSimpleCameraCalTInterface(const std::string& transform_frame, const std::string& parent_frame);

  /**
   * @brief Default destructor
   */
  ~ROSSimpleCameraCalTInterface(){};

  /** @brief  uses the pose to compute the new joint values, and sends them to the mutable joint state publisher
   * @param Pose the pose is the transform from the parent frame to the transform
   */
  bool pushTransform(Pose6d& Pose);

  /** @brief get the transform from the mutable transform publisher, and compute the optical to reference frame pose*/
  Pose6d pullTransform();

  /** @brief as a listener interface, tells the mutable transform publisher to store its current values in its yaml file
   */
  bool store(std::string& filePath);

  /** @brief sets the reference frame of the transform interface, sometimes not used */
  void setReferenceFrame(std::string& ref_frame);

private:
  ros::NodeHandle* nh_;
  std::string transform_frame_;   /**< transform frame name */
  std::string parent_frame_;      /**< parent's frame name */
  Pose6d pose_;                   /**< pose associated with the transform from reference frame to housing frame */
  ros::ServiceClient get_client_; /**< a client for calling the service to get the joint values associated with the
                                     transform */
  ros::ServiceClient set_client_; /**< a client for calling the service to set the joint values associated with the
                                     transform */
  ros::ServiceClient
      store_client_; /**< a client for calling the service to store the joint values associated with the transform */
  std::vector<std::string> joint_names_; /**< names of joints  */
  std::vector<double> joint_values_;     /**< values of joints  */
  industrial_extrinsic_cal::get_mutable_joint_states::Request get_request_;
  industrial_extrinsic_cal::get_mutable_joint_states::Response get_response_;
  industrial_extrinsic_cal::set_mutable_joint_states::Request set_request_;
  industrial_extrinsic_cal::set_mutable_joint_states::Response set_response_;
  industrial_extrinsic_cal::store_mutable_joint_states::Request store_request_;
  industrial_extrinsic_cal::store_mutable_joint_states::Response store_response_;
};

}  // end industrial_extrinsic_cal namespace
#endif /* ROS_CAMERA_OBSERVER_H_ */
