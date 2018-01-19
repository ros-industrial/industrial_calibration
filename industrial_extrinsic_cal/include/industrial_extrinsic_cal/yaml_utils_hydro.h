#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/trigger.h>

namespace industrial_extrinsic_cal
{
#define YAML_ITERATOR YAML::Iterator

inline bool parseDouble(const YAML::Node& node, char const* var_name, double& var_value)
{
  if (node.FindValue(var_name))
  {
    node[var_name] >> var_value;
    return true;
  }
  return false;
}

inline bool parseInt(const YAML::Node& node, char const* var_name, int& var_value)
{
  if (node.FindValue(var_name))
  {
    node[var_name] >> var_value;
    return true;
  }
  return false;
}
inline bool parseUInt(const YAML::Node& node, char const* var_name, unsigned int& var_value)
{
  if (node.FindValue(var_name))
  {
    node[var_name] >> var_value;
    return true;
  }
  return false;
}

inline bool parseString(const YAML::Node& node, char const* var_name, std::string& var_value)
{
  if (node.FindValue(var_name))
  {
    node[var_name] >> var_value;
    return true;
  }
  return false;
}

inline bool parseBool(const YAML::Node& node, char const* var_name, bool& var_value)
{
  if (node.FindValue(var_name))
  {
    node[var_name] >> var_value;
    return true;
  }
  return false;
}

inline bool parseVectorD(const YAML::Node& node, char const* var_name, std::vector<double>& var_value)
{
  if (node.FindValue(var_name))
  {
    var_value.clear();
    const YAML::Node* n = node.FindValue(var_name);
    for (int i = 0; i < (int)n->size(); i++)
    {
      double value;
      (*n)[i] >> value;
      var_value.push_back(value);
    }
    return true;
  }
  return false;
}

inline const YAML::Node& parseNode(const YAML::Node& node, char const* var_name)
{
  if (!node.FindValue(var_name))
  {
    ROS_ERROR("Can't parse node %s", var_name);
    return (YAML::Node());
  }
  return (node[var_name]);
}

inline void parseKeyDValue(YAML::Iterator& it, std::string& key, double& dvalue)
{
  it.first() >> key;
  it.second() >> dvalue;
}

inline bool yamlNodeFromFileName(std::string filename, YAML::Node& ynode)
{
  bool rtn;
  std::ifstream fs;
  fs.open(filename.c_str());
  if (fs.fail())
  {
    ROS_ERROR("could not open %s in yamlNodeFromFileName()", filename.c_str());
    rtn = false;
  }
  else
  {
    YAML::Parser parser(fs);
    parser.GetNextDocument(ynode);
    rtn = true;
  }
  return (rtn);
}

}  // end namespace

#endif
