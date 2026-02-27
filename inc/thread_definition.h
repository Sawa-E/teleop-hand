#ifndef MOTIONCONTROL_THREAD_DEFINITION_H
#define MOTIONCONTROL_THREAD_DEFINITION_H

namespace mc {
namespace thread {
  enum thread_list
  {
    compute_engine,
    read_sensor,
    write_output,
    record_motion,
    recv_command,
    draw_gui,
    thread_list_size
  };
} // namespace thread
} // namespace mc
#endif //MOTIONCONTROL_THREAD_DEFINITION_H
