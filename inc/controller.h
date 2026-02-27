#ifndef MOTIONCONTROL_CONTROLER_H
#define MOTIONCONTROL_CONTROLER_H
#include "robot_system.h"
#include <cmath>

namespace mc {
  class control
  {
  using controller_method = void (*)(robot_system &robot);

  public:
    control()
    {
      for (int i = 0 ; i < mc::control_mode_size; ++i)
      {
        controller[i] = mc::control::default_controller;
      }
      register_controller();
    }

    void register_controller();

    std::array<controller_method, mc::control_mode_size> controller;
  private:
    static void default_controller(robot_system &robot);
  };
} // namespace mc
#endif //MOTIONCONTROL_CONTROLER_H
