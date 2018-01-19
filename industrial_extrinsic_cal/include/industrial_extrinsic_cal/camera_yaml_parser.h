#ifndef CAMERA_YAML_PARSER_H
#define CAMERA_YAML_PARSER_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/yaml_utils.h>

namespace industrial_extrinsic_cal
{
// prototypes
/** @brief parses a single camera, either moving or static
 *   @param node, the yaml node
 *   @return a shared pointer to camera parsed
 **/
boost::shared_ptr<Camera> parseSingleCamera(const YAML::Node& node);
/** @brief parses a transform interface
 *   @param node, the yaml node
 *   @param frame, the frame that the transform interface refers to
 *   @param pose, the initial value of the pose
 *   @return a shared pointer to transform interface parsed
 **/
boost::shared_ptr<TransformInterface> parseTransformInterface(const YAML::Node& node, std::string& name,
                                                              std::string& frame, Pose6d& pose);
/** @brief parses a trigger
 *   @param node, the yaml node
 *   @param name name of trigger
 *   @return shared pointer to the trigger parsed
 **/
boost::shared_ptr<Trigger> parseTrigger(const YAML::Node& node, std::string& name);
/** @brief parses a Pose6d
 *   @param node, the yaml node
 *   @param pose as 6dof pose from parsed position and angle axis parameters xyz,wx,wy,wz
 *   @return true if sucessful
 **/
bool parsePose(const YAML::Node& node, Pose6d& pose);
/** @brief parse all the cameras, the main function of this module
 *   @param camera_input_file path/file from which to parse a vector of cameras
 *   @param returned vector of shared pointers to cameras
 *   @return true if successful
 **/
bool parseCameras(std::string& cameras_input_file, std::vector<boost::shared_ptr<Camera> >& cameras);

}  // end industrial_extrinsic_cal namespace

#endif
