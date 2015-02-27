#ifndef CAMERA_YAML_PARSER_H
#define CAMERA_YAML_PARSER_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/trigger.h>


namespace industrial_extrinsic_cal {

  using std::string;
  using std::ifstream;
  using std::vector;
  using boost::shared_ptr;
  using YAML::Node;

  // prototypes
  shared_ptr<Camera> parseSingleCamera(const Node &node);
  shared_ptr<TransformInterface> parseTransformInterface(const Node &node, string &name, string &frame);
  shared_ptr<Trigger> parseTrigger(const Node &node, string &name);
  Pose6d parsePose(const Node &node);
  void parseCameras(ifstream &cameras_input_file,vector<shared_ptr<Camera> >& cameras);
}// end industrial_extrinsic_cal namespace

#endif
