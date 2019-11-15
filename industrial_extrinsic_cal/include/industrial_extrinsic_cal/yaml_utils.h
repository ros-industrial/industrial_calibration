#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <ros/package.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/trigger.h>

namespace industrial_extrinsic_cal
{
#define YAML_ITERATOR YAML::const_iterator

inline bool parseDouble(const YAML::Node& node, char const* var_name, double& var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<double>();
    return true;
  }
  return false;
}

inline bool parseInt(const YAML::Node& node, char const* var_name, int& var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<int>();
    return true;
  }
  return false;
}
inline bool parseUInt(const YAML::Node& node, char const* var_name, unsigned int& var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<unsigned int>();
    return true;
  }
  return false;
}

inline bool parseString(const YAML::Node& node, char const* var_name, std::string& var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<std::string>();
    return true;
  }
  return false;
}

inline bool parseBool(const YAML::Node& node, char const* var_name, bool& var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name];
    return true;
  }
  return false;
}

inline bool parseVectorD(const YAML::Node& node, char const* var_name, std::vector<double>& var_value)
{
  if (node[var_name])
  {
    var_value.clear();
    const YAML::Node n = node[var_name];
    for (int i = 0; i < (int)n.size(); i++)
    {
      double value;
      value = n[i].as<double>();
      var_value.push_back(value);
    }
    return true;
  }
  return false;
}

inline const YAML::Node parseNode(const YAML::Node& node, char const* var_name)
{
  if (!node[var_name])
  {
    ROS_INFO("Can't parse node[%s]", var_name);
  }
  return (node[var_name]);
}
inline void parseKeyDValue(YAML::const_iterator& it, std::string& key, double& dvalue)
{
  key = it->first.as<std::string>();
  dvalue = it->second.as<double>();
}

inline bool yamlNodeFromFileName(std::string filename, YAML::Node& ynode)
{
  bool rtn = true;
  try
  {
    ynode = YAML::LoadFile(filename.c_str());
  }
  catch (int e)
  {
    ROS_ERROR("could not open %s in yamlNodeFromFileName()", filename.c_str());
    rtn = false;
  }
  return (rtn);
}

inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      return std::string();
    }
  }

  return mod_url;
}

}  // end namespace

#endif
