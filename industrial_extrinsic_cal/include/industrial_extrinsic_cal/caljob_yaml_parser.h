#ifndef CALJOB_YAML_PARSER_H
#define CALJOB_YAML_PARSER_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/yaml_utils.h>
#include <industrial_extrinsic_cal/observation_scene.h>
#include <industrial_extrinsic_cal/ceres_blocks.h>

namespace industrial_extrinsic_cal
{
// prototypes
/** @brief parses the calibration job definition yaml file
 *   @param caljob_input_file a stream pointing at the file
 *   @param scene_list the returned list of scenes
 *   @param reference_frame the retured reference frame
 *   @param blocks the blocks containing pointers to all the camera and target parameters for ceres
 **/
bool parseCaljob(std::string& caljob_input_file, std::vector<ObservationScene>& scene_list,
                 std::string& reference_frame, CeresBlocks& blocks);

/** @brief parses a single scene
 *   @param node, the yaml node
 *   @param scene_id the current scenes' id
 *   @param blocks the blocks containing pointers to all the camera and target parameters for ceres
 *   @return an observation scene
 **/
ObservationScene parseSingleScene(const YAML::Node& node, int scene_id, CeresBlocks& blocks);
}  // end industrial_extrinsic_cal namespace

#endif
