#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/transform_interface.hpp>
#include <industrial_extrinsic_cal/trigger.h>

namespace industrial_extrinsic_cal 
{
  
  inline bool parseDouble(const YAML::Node &node, char const * var_name, double &var_value)
  {
    node[var_name] >> var_value;
  }

  inline bool parseInt(const YAML::Node &node, char const * var_name, int &var_value)
  {
    node[var_name] >> var_value;
  }
  inline bool parseUInt(const YAML::Node &node, char const * var_name, unsigned int &var_value)
  {
    node[var_name] >> var_value;
  }

  inline bool parseString(const YAML::Node &node, char const * var_name, std::string &var_value)
  {
    node[var_name] >> var_value;
  }

  inline bool parseBool(const YAML::Node &node, char const * var_name, bool &var_value)
  {
    node[var_name] >> var_value;
  }

  inline bool parseVectorD(const YAML::Node &node, char const * var_name, std::vector<double> &var_value)
  {
    node[var_name] >> var_value;
  }

  inline const YAML::Node * parseNode(const YAML::Node &node, char const * var_name)
  {
    return(node.FindValue(var_name));
  }
  inline void parseKeyDValue(YAML::Iterator &it, std::string &key, double &dvalue)
    {
      it.first() >> key;
      it.second() >> dvalue;
    }
} //end namespace

#endif
