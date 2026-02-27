#ifndef MOTIONCONTROL_JOINT_H
#define MOTIONCONTROL_JOINT_H

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include "joint_parameter_definition.h"
#include "enum_helper.h"

template<typename T>
struct joint
{
public:
  //data
  T data[mc::type_size][mc::state_size];
  T data_buf[mc::type_size][mc::state_size];
  T parameter[mc::parameter_size];

  //method
  joint(const char * config_file_name)
  :config_file_name(config_file_name)
  {
    std::cout << "constructor robot_system" << std::endl;
    joint_id = number_of_joints++;
    reset();
    update_parameter();
  }
  int get_joint_id() const
  {
    return joint_id;
  }
  template<typename U>
  friend std::ostream& operator << (std::ostream& os, const joint<U>& j);

private:
  // data
  std::string config_file_name;
  int joint_id;
  static int number_of_joints;

  //method
  T get_value_from_json(const std::string key, const boost::property_tree::ptree &parameter)
  {
    if (boost::optional<T> value = parameter.get_optional<T>(key))
      return value.get();
    else
    {
      std::cerr << "[Warning]\n"
                << "\tKeyError: no such kind of parameter as "
                << "\""
                << key
                << "\""
                << std::endl
                << "\tSet default value"
                << std::endl;
      return 0.0;//TODO 0.0 is not suitable for template
    }
  }
  void reset()
  {
    for (int i = 0; i < mc::type_size; ++i)
    {
      for (int j = 0; j < mc::state_size; j++)
      {
        data[i][j] = 0;
        data_buf[i][j] = 0;
      }
    }

    for (int i = 0; i < mc::parameter_size; ++i)
      parameter[i] = 0;
    std::cout << "state and parameter is reseted" << std::endl;
  }

  void update_parameter();
};

template<typename T>
int joint<T>::number_of_joints = 0;

template<typename T>
std::ostream& operator << (std::ostream& os, const joint<T>& j)
{
  os << "joint id: " << j.get_joint_id() << std::endl;
  os << "paramers: " << std::endl;
  for (int i = 0 ; i < mc::parameter_size; ++i)
  {
    os << "\t" << j.parameter[i] << std::endl;
  }
   return os;
};

template<typename T>
void joint<T>::update_parameter()
{
  boost::property_tree::ptree pt;
  read_json(this->config_file_name.c_str(), pt);

  /*
    paramer defined mc::parameter in inc/joint_parameter_definition.h are automatically updated based on json config file.
    if column is not existed in json config file, default value(0) is replaced.
  */
  for (int i = 0; i < mc::parameter_size; ++i)
  {
    const auto key = static_cast<mc::parameter>(i);
    parameter[key] = get_value_from_json(mc::enum_helper::name(key), pt);
  }
}
#endif //MOTIONCONTROL_JOINT_H
