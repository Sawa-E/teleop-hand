#ifndef MOTIONCONTROL_ENUM_HELPER_H
#define MOTIONCONTROL_ENUM_HELPER_H
#include <iostream>
#include <string>
#include <boost/preprocessor.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>

#define CASE(z, n, text) case n:ret = nameof_impl < E, E{ static_cast<E>(n) } > ();break;

namespace mc {
  class enum_helper
  {
  private:
    template <typename E>
    static size_t get_type_length()
    {
        constexpr const char* str = __PRETTY_FUNCTION__;
        constexpr auto str_len = sizeof(__PRETTY_FUNCTION__);
        constexpr auto prefix_len = sizeof("size_t get_type_length() [") - 1;
        constexpr auto suffix_len = sizeof("; size_t = long unsigned int]");
        std::string str_ = std::string{ str + prefix_len,str + str_len - suffix_len };
        return std::string{ str + prefix_len,str + str_len - suffix_len }.length();
    }

    template <typename E, E V>
    static std::string nameof_impl()
    {
        constexpr auto str = __PRETTY_FUNCTION__;
        constexpr auto prefix_len = sizeof("std::string nameof_impl() [ E V = mc::") - 1;
        constexpr auto suffix_len = sizeof("; std::string = std::__cxx11::basic_string<char>]");
        constexpr auto size = sizeof(__PRETTY_FUNCTION__);
        return std::string{ str + prefix_len + get_type_length<E>() + 1,str + size - suffix_len };
    }

  public:
    template <typename E>
    static std::string name(E e)
    {
        std::string ret;
        switch (e) {
        BOOST_PP_REPEAT(30, CASE, _);
        default:
            ret = "";
            break;
        }
        //std::string str = ret;
        auto pos = ret.find_last_of("::");
        if (pos != std::string::npos)
          ret = ret.substr(pos + 1);
        return ret;
    }
  };
} // namespace mc
#undef CASE
#endif //MOTIONCONTROL_ENUM_HELPER_H
