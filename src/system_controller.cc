#include "system_controller.h"
#include "control_timer.h"

#define   x_res(n) robot_.joints[(n)].data[mc::response][mc::x]
#define  dx_res(n) robot_.joints[(n)].data[mc::response][mc::dx]
#define ddx_res(n) robot_.joints[(n)].data[mc::response][mc::ddx]
#define   x_buf(n) robot_.joints[(n)].data_buf[mc::response][mc::x]
#define  dx_buf(n) robot_.joints[(n)].data_buf[mc::response][mc::dx]
#define ddx_buf(n) robot_.joints[(n)].data_buf[mc::response][mc::ddx]
#define f_out(n) robot_.joints[(n)].data[mc::output][mc::f]
#define f_dis(n) robot_.joints[(n)].data[mc::response][mc::f_dis]
#define f_vol(n) robot_.joints[(n)].data[mc::output][mc::f_dis]
#define f_buf(n) robot_.joints[(n)].data_buf[mc::response][mc::f_dis]
#define M_n(n) robot_.joints[(n)].parameter[mc::mass]
#define D_n(n) robot_.joints[(n)].parameter[mc::damper]
#define K_n(n) robot_.joints[(n)].parameter[mc::spring]
#define g_diff(n) robot_.joints[(n)].parameter[mc::g_diff]
#define g_dis(n) robot_.joints[(n)].parameter[mc::g_dis]

using ll = long long int;
using namespace mc;
using namespace mc::thread;
void system_controller::task_registration()
{
  tasks_[::compute_engine] = [this](ll sample_frequency)
  {
    (void)(sample_frequency);
    if (robot_.is_control_mode_changed())
    {
      robot_.reset_control_step();
      robot_.update_control_mode();
    }

    // get latest finger data from UDP receiver
    {
      auto [valid, fd] = udp_receiver_.get_latest_data();
      if (valid)
      {
        robot_.set_to_dict("distance_mm", static_cast<double>(fd.distance_mm));
        robot_.set_to_dict("packet_count", static_cast<double>(udp_receiver_.get_packet_count()));
        robot_.set_to_dict("last_receive_time", static_cast<double>(udp_receiver_.get_last_receive_time()));
      }
    }

    // calculate force reference for controling robot
    calculate_output_command();
    return system_controller::ON;
  };

  tasks_[::read_sensor] = [this](ll sample_frequency)
  {
    // read signal from sensors
    static long long int cnt;
    double dt = robot_.get_from_dict("dt");
    cnt++;

    // update counter pulse for each joint
    for (size_t i = 0; i < readers_ptr_.size(); ++i)
      readers_ptr_[i]->read();
    update_all_position();

    for (size_t i = 0; i < robot_.joints.size(); ++i)
    {
      mc::signal<double>::pseudo_differential(dt, g_diff(i), dx_res(i), x_res(i), x_buf(i));  // update dx
      mc::signal<double>::pseudo_differential(dt, g_diff(i), ddx_res(i), dx_res(i), dx_buf(i));// update ddx
      mc::signal<double>::disturbance_observer(dt, g_dis(i), f_dis(i), f_vol(i), dx_res(i), f_buf(i), M_n(i));
    }
    return system_controller::ON;
  };

  tasks_[::write_output] = [this](ll sample_frequency)
  {
    (void)(sample_frequency);
    // output command to devices
    static long long int cnt;
    cnt++;
    for (size_t i = 0; i < writers_ptr_.size(); ++i)
    {
      writers_ptr_[i]->write();
    }
    return system_controller::ON;
  };

  tasks_[::record_motion] = [this](ll sample_frequency)
  {
    (void)(sample_frequency);
    static FILE *fp = nullptr;
    static bool was_recording = false;

    bool should_record = (robot_.get_control_mode() == mc::Record);

    if (should_record && !was_recording)
    {
      // start recording
      fp = record_open();
      was_recording = true;
      robot_.is_recording = true;
    }
    else if (!should_record && was_recording)
    {
      // stop recording
      if (fp != nullptr)
      {
        fclose(fp);
        fp = nullptr;
      }
      was_recording = false;
      robot_.is_recording = false;
    }

    if (was_recording && fp != nullptr)
    {
      record_line(fp, robot_);
    }

    // ON: record_motion runs as is_single_loop=true (realtime periodic task)
    return system_controller::ON;
  };

  tasks_[::recv_command] = [this](ll sample_frequency)
  {
    (void)(sample_frequency);
    udp_receiver_.receive();
    return system_controller::ON;
  };

  tasks_[::draw_gui] = [this](ll sample_frequency)
  {
    (void)(sample_frequency);
    std::cout << "class gui" << std::endl;
    gui_.bind_robot_system_ptr(&robot_);

    gui_.register_widget("display_status", widget::display_status, true);
    gui_.register_widget("control_mode_selection", widget::control_mode_selection, true);
    gui_.register_widget("plot_state", widget::plot_state, true);
    gui_.register_widget("finger_tracker_status", widget::finger_tracker_status, true);

    // the draw method will holding thread until 'exit' button or 'q' are pressed.
    if (gui_.draw() == gui::SUCCEESS)
      std::printf("gui is successfully finished\n");
    else
      std::printf("gui process is failed\n");
    return system_controller::FINISH;
  };
}

void system_controller::record_line(FILE *fp, robot_system &robot)
{
  static long long int time;
  time = control_timer::get_micro_time();
  fprintf(fp, "%d, %lld", robot.get_control_mode(), time);
  for (size_t i = 0; i < robot.joints.size(); ++i)
  {
    fprintf(fp, ",%lf,%lf,%lf,%lf",
      robot.joints[i].data[mc::response][mc::x],
      robot.joints[i].data[mc::response][mc::dx],
      robot.joints[i].data[mc::response][mc::f_dis],
      robot.joints[i].data[mc::output][mc::f]
    );
  }
  // append theta_cmd and distance_mm
  fprintf(fp, ",%lf,%lf",
    robot.get_from_dict("theta_cmd"),
    robot.get_from_dict("distance_mm")
  );
  fprintf(fp, "\n");
}
