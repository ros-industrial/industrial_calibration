#include <stdlib.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/trigger.h>
#include <industrial_extrinsic_cal/ros_triggers.h>
#include <industrial_extrinsic_cal/camera_yaml_parser.h>
#include <industrial_extrinsic_cal/caljob_yaml_parser.h>
#include <industrial_extrinsic_cal/yaml_utils.h>

namespace industrial_extrinsic_cal
{
using std::ifstream;
using std::string;
using std::vector;
using boost::shared_ptr;
using boost::make_shared;
using YAML::Node;

bool parseCaljob(std::string& caljob_input_file, vector<ObservationScene>& scene_list, string& reference_frame,
                 CeresBlocks& blocks)
{
  bool rtn = true;
  try
  {
    Node caljob_doc;
    if (!yamlNodeFromFileName(caljob_input_file, caljob_doc))
    {
      ROS_ERROR("Can't parse yaml file %s", caljob_input_file.c_str());
    }

    // get reference frame
    if (!parseString(caljob_doc, "reference_frame", reference_frame))
    {
      ROS_ERROR("must set caljob's reference frame");
    }

    // read in all scenes
    scene_list.clear();
    const YAML::Node& scenes_node = parseNode(caljob_doc, "scenes");
    for (unsigned int i = 0; i < scenes_node.size(); i++)
    {
      ObservationScene temp_scene = parseSingleScene(scenes_node[i], i, blocks);
      temp_scene.setSceneId(i);
      scene_list.push_back(temp_scene);
    }
    ROS_INFO_STREAM((int)scene_list.size() << " scenes");
  }
  catch (YAML::ParserException& e)
  {
    ROS_ERROR("Caljob parsing failure");
    rtn = false;
  }
  return (rtn);
}
ObservationScene parseSingleScene(const Node& node, int scene_id, CeresBlocks& blocks)
{
  ObservationScene temp_scene;
  try
  {
    // parse the scene trigger
    std::string trigger_name;
    if (!parseString(node, "trigger", trigger_name)) ROS_ERROR("no trigger in scene");
    boost::shared_ptr<Trigger> temp_trigger = parseTrigger(node, trigger_name);
    temp_scene.setTrigger(temp_trigger);

    // parse the observations
    const YAML::Node& observation_node = parseNode(node, "observations");
    for (int i = 0; i < (int)observation_node.size(); i++)
    {
      std::string camera_name;
      std::string target_name;
      std::string cost_type_string;
      Cost_function cost_type;
      shared_ptr<Camera> temp_cam = make_shared<Camera>();
      shared_ptr<Target> temp_targ = make_shared<Target>();
      Roi temp_roi;
      if (!parseString(observation_node[i], "camera", camera_name)) ROS_ERROR("no camera_name in observation");
      if (!parseString(observation_node[i], "target", target_name)) ROS_ERROR("no target in observation");
      if (!parseString(observation_node[i], "cost_type", cost_type_string)) ROS_ERROR("no cost_type in observation");
      cost_type = string2CostType(cost_type_string);
      if (!parseInt(observation_node[i], "roi_x_min", temp_roi.x_min))
        ROS_ERROR("need to define roi_x_min in observation");
      if (!parseInt(observation_node[i], "roi_x_max", temp_roi.x_max))
        ROS_ERROR("need to define roi_x_max in observation");
      if (!parseInt(observation_node[i], "roi_y_min", temp_roi.y_min))
        ROS_ERROR("need to define roi_y_min in observation");
      if (!parseInt(observation_node[i], "roi_y_max", temp_roi.y_max))
        ROS_ERROR("need to define roi_y_max in observation");
      if ((temp_cam = blocks.getCameraByName(camera_name)) == NULL)
      {
        ROS_ERROR("Couldn't find camea %s", camera_name.c_str());
      }
      if ((temp_targ = blocks.getTargetByName(target_name)) == NULL)
      {
        ;
        ROS_ERROR("Couldn't find target %s", target_name.c_str());
      }
      if (scene_id != 0 && temp_targ->is_moving_)
      {  // if we have a moving target, we need to add it to the blocks
        blocks.addMovingTarget(temp_targ, scene_id);
        temp_targ = blocks.getTargetByName(target_name, scene_id);
      }
      temp_scene.addCameraToScene(temp_cam);
      temp_scene.populateObsCmdList(temp_cam, temp_targ, temp_roi, cost_type);
    }
  }
  catch (YAML::ParserException& e)
  {
  }
  return (temp_scene);
}

}  // end of industrial_extrinsic_cal namespace
