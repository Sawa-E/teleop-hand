#ifndef MOTIONCONTROL_LAYOUT_H
#define MOTIONCONTROL_LAYOUT_H
#include <unordered_map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using Vec=ImVec2;

namespace mc
{
  struct widget_layout
  {
    std::vector<int> top_left;
    std::vector<int> bottom_right;
  };

  class layout
  {
  public:
    layout(const char* layout_file)
    {
      boost::property_tree::read_json(layout_file, pt_);
      std::cout << "Start to read layout json file" << std::endl;
      if (auto tree = pt_.get_optional<int>("grid.col_size"))
        col_size_ = tree.get();
      else
        error();
      if (auto tree = pt_.get_optional<int>("grid.row_size"))
        row_size_ = tree.get();
      else
        error();
      for (auto child: pt_.get_child("widget"))
      {
        std::vector<int> tl;
        std::vector<int> br;
        std::string name;
        auto record =  child.second;
        if (auto tree = record.get_optional<std::string>("name"))
          name = tree.get();
        else
          error();

        if (record.get_child("top_left").size() == 0 || record.get_child("bottom_right").size() == 0)
        {
          error();
        }
        else
        {
          for (auto &vec : record.get_child("top_left"))
            tl.push_back(vec.second.get_optional<int>("").get());
          for (auto &vec : record.get_child("bottom_right"))
            br.push_back(vec.second.get_optional<int>("").get());
          widgets_[name] = {tl, br};

          if(tl.size() != 2 || br.size() != 2)
            error();
        }
      }
      std::cout << "finish to read layout json file" << std::endl;
    }

    mc::widget_layout operator[](const char *key)
    {
      if (widgets_.count(key) == 0)
        error();
      return widgets_[key];
    }

    Vec get_position(int width, int height, const char *key)
    {
      int x(DEFAULT_X_),y(DEFAULT_Y_);
      if (widgets_.count(key) != 0)
      {
        int dw = width / col_size_;
        int dh = height / row_size_;
        x = dw * widgets_[key].top_left[0];
        y = dh * widgets_[key].top_left[1];
      }
      return Vec(x,y);
    }

    Vec get_size(int width, int height, const char *key)
    {
      int w(DEFAULT_W_),h(DEFAULT_H_);
      if (widgets_.count(key) != 0)
      {
        int dw = width / col_size_;
        int dh = height / row_size_;
        w = dw * (widgets_[key].bottom_right[0] - widgets_[key].top_left[0] + 1);
        h = dh * (widgets_[key].bottom_right[1] - widgets_[key].top_left[1] + 1);
      }
      return Vec(w,h);
    }
  private:
    boost::property_tree::ptree pt_;
    std::unordered_map<std::string, widget_layout> widgets_;
    int col_size_;
    int row_size_;
    const int DEFAULT_H_ = 100;
    const int DEFAULT_W_ = 100;
    const int DEFAULT_X_ = 100;
    const int DEFAULT_Y_ = 100;

    // method
    void error()
    {
      std::cerr << "Error:" << std::endl;
      std::cerr << "\t error in layout configuration" << std::endl;
      std::cerr << "\t maybe you should check your json file for layout" << std::endl;
    }
  };
} // namespace mc

#endif //MOTIONCONTROL_LAYOUT_H
