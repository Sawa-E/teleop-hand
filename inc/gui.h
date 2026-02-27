#ifndef MOTIONCONTROL_GUI_H
#define MOTIONCONTROL_GUI_H
#include <unistd.h>
#include <iostream>
#include <unordered_map>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "gui_widget.h"
#include "robot_system.h"
#include "layout.h"


GLFWwindow *reset_and_open_window(const std::string window_name);
void initialize_gui(GLFWwindow *window);
void clean_up_window(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

class gui
{
  using gui_widget_method = void (*)(robot_system *robot_system_ptr);
public:
  // data
  static const int SUCCEESS = 0;
  static const int FAIL = -1;

  // method
  gui(const char *window_name)
  :window_name_(window_name),
  layout_("../config/layout.json")
  {
    std::cout << "constructor gui" << std::endl;
  }

  int draw()
  {
    window_ptr_ = reset_and_open_window(window_name_.c_str());
    initialize_gui(window_ptr_);
    while (!glfwWindowShouldClose(window_ptr_))
    {
      draw_windows();
      usleep(SLEEP_MICRO_TIME);
    }
    clean_up_window(window_ptr_);
    return SUCCEESS;
  }

  int register_widget(const char * name, gui_widget_method method, bool initial_state)
  {
    if (name_to_show_flag_and_widget.count(name) == 0)
    {
      name_to_show_flag_and_widget[name] = std::make_pair(initial_state, method);
      std::cout << "widget: " << name << " is registered" << std::endl;
      return SUCCEESS;
    }
    else
    {
      std::cerr << "You can not register widget on this name!!" << std::endl;
      return FAIL;
    }
  }

  void bind_robot_system_ptr(robot_system *robot_system_ptr)
  {
    robot_system_ptr_ = robot_system_ptr;
  }
private:
  // data
  static const int SLEEP_MICRO_TIME = 20000;
  std::string window_name_;
  GLFWwindow *window_ptr_;
  std::unordered_map<std::string, std::pair<bool, gui_widget_method>> name_to_show_flag_and_widget;
  robot_system *robot_system_ptr_;
  mc::layout layout_;

  //method
  void show_widgets(const std::string &widget_name);
  void draw_windows();
};
#endif //MOTIONCONTROL_GUI_H
