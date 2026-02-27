#ifndef MOTIONCONTROL_ENVIRONMENT_H
#define MOTIONCONTROL_ENVIRONMENT_H
#include <iostream>
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

namespace mc {
  class environment
  {
  public:
    environment(const char *file_name)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_json(file_name, pt);
      std::cout << "setup environment variables from " << file_name << " json file." << std::endl;
      for (auto& child : pt.get_child(""))
      {
        std::string key, value;
        key = child.first;
        value = child.second.data();
        parameter[key] = value;
      }
    }

    std::string operator[](std::string key)
    {
      if (parameter.count(key) == 0)
      {
        return "";
      }
      else
      {
        std::string ret = parameter[key];
        return ret;
      }
    }
  private:
    std::unordered_map<std::string, std::string> parameter;
  };

  namespace env {
    static environment variable("../config/system.json");
  } // namespace variable
} // namespace mc
#endif //MOTIONCONTROL_ENVIRONMENT_H
