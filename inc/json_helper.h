#ifndef MOTIONCONTROL_JSON_HELPER_H
#define MOTIONCONTROL_JSON_HELPER_H
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

template<typename T>
class json_helper
{
public:
  static T get_value_from_json(const std::string key, const boost::property_tree::ptree &parameter)
  {
    if (boost::optional<T> value = parameter.get_optional<T>(key))
      return value.get();
    else
    {
      std::cerr << "[Error]\n"
                << "\tKeyError: no such kind of parameter as "
                << "\""
                << key
                << "\""
                << std::endl
                << "\tExit"
                << std::endl;
      exit(1);
    }
  }
};
#endif //MOTIONCONTROL_JSON_HELPER_H

