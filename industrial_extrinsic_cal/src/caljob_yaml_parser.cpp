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

using std::ifstream;
using std::string;
using std::vector;
using boost::shared_ptr;
using boost::make_shared;
using YAML::Node;

namespace industrial_extrinsic_cal {

  bool parseCaljob(ifstream &caljob_input_file, std:vector<<ObservationScene> > &scene_list, std::string *reference_frame, CeresBlocks & blocks)
  {
    YAML::Parser _parser(caljob_input_file);
    Node caljob_doc;
    bool rtn=true;
    try{
      caljob_parser.GetNextDocument(camera_doc);

      // get reference frame       
      if (!parseString("reference_frame", reference_frame)){
	ROS_ERROR("must set caljob's reference frame");
      }

      // read in all scenes
      scene_list.clear();
      if (const Node *scenes_node = parseNode(camera_doc, "scenes") ){
	ROS_INFO_STREAM("Found "<<scenes_node->size()<<" scenes ");
	for (unsigned int i = 0; i < scenes_node->size(); i++){
	  ObservationScene temp_scene = parseSingleScene((*scenes_node)[i], i, blocks);
	  temp_scene.setSceneId(i);
	  scene_list.push_back(temp_scene);
	}
      }      // end if there are any scenes
      else{
	ROS_INFO("no scenes");
      }
      ROS_INFO_STREAM("Successfully read in " << (int) scene_list.size() << " scenes");
    }
    catch (YAML::ParserException& e){
      ROS_ERROR("Caljob parsing failure");
      rtn = false;
    }
    return(rtn);
  }
  ObservationScene parseSingleScene(const Node &node, int scene_id, CeresBlocks & blocks)
  {
    ObservationScene temp_scene;
    try{
      // parse the trigger
      parseString(node, "trigger", trigger_name);
      boost::shared_ptr<Trigger> temp_trigger = parseTrigger(node, trigger_name);
      temp_scene.setTrigger(temp_trigger);

      // parse the observations
      const YAML::Node *observation_node = parseNode(node, "observations");
      for(int i=0; i<(int)observation_node->size();i++){
	std::string camera_name;
	std::string target_name;
	std::string cost_type_string;
	Cost_function cost_type;
	shared_ptr<Camera> temp_cam = make_shared<Camera>();
	shared_ptr<Target> temp_targ = make_shared<Target>();
	Roi temp_roi;
	bool success = true;
	success &= parseString("camera", camera_name);
	success &= parseString("target", target_name);
	success &= parseString("cost_type", cost_type_string);
	cost_type = string2CostType(cost_type_string);
	success &= parseUInt("roi_x_min", temp_roi.x_min);
	success &= parseUInt("roi_x_max", temp_roi.x_max);
	success &= parseUInt("roi_y_min", temp_roi.y_min);
	success &= parseUInt("roi_y_max", temp_roi.y_max);
	
	if((temp_cam = blocks.getCameraByName(camera_name)) == NULL){
	  ROS_ERROR("Couldn't find camea %s",camera_name.c_str());
	}
	if((temp_targ = blocks.getTargetByName(target_name)) == NULL){;
	  ROS_ERROR("Couldn't find target %s",target_name.c_str());
	}
	if(scene_id !=0 && temp_targ->is_moving_){ // if we have a moving target, we need to add it to the blocks
	  blocks.addMovingTarget(temp_targ, scene_id);
	  temp_targ =  blocks.getTargetByName(target_name, scene_id);
	}
	temp_scene.addCameraToScene(temp_cam);
	temp_scene.populateObsCmdList(temp_cam, temp_targ, temp_roi, cost_type);
      }
    }
    catch (YAML::ParserException& e){
    }
    return(rtn);
  }
  

}// end of industrial_extrinsic_cal namespace
