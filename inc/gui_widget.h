#ifndef TELEOPHAND_GUI_WIDGET_H
#define TELEOPHAND_GUI_WIDGET_H
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "robot_system.h"
#include "implot.h"
#include "enum_helper.h"
#include "thread_definition.h"

namespace mc {
  struct scrolling_buffer
  {
    int max_size;
    int offset;
    ImVector<ImVec2> data;
    scrolling_buffer(int max_size = 2000) {
      this->max_size = max_size;
      offset  = 0;
      data.reserve(max_size);
    }
    void add_point(float x, float y) {
      if (data.size() < max_size)
        data.push_back(ImVec2(x,y));
      else {
        data[offset] = ImVec2(x,y);
        offset =  (offset + 1) % max_size;
      }
    }
    void erase() {
      if (data.size() > 0) {
        data.shrink(0);
        offset  = 0;
      }
    }
  };

  // shared gui time for synchronized plot axes
  static inline float gui_time = 0.0f;

  class widget
  {
  public:
    static void display_status(robot_system *robot_system_ptr)
    {
      ImGui::Columns(3, "takt time");
      ImGui::Text("Mode: %d ", robot_system_ptr->get_control_mode());
      ImGui::NextColumn();
      ImGui::Text("Compute: %lld [us]", robot_system_ptr->task_takt_times[mc::thread::compute_engine]);
      ImGui::NextColumn();
      ImGui::Text("Sensor: %lld [us]", robot_system_ptr->timer_sampling_times[mc::thread::read_sensor]);
      ImGui::Separator();
      ImGui::Columns(1);

      ImGui::Columns(5, "motion_header_column");
      ImGui::Text(" ");
      ImGui::NextColumn();
      ImGui::Text("position");
      ImGui::NextColumn();
      ImGui::Text("velocity");
      ImGui::NextColumn();
      ImGui::Text("force");
      ImGui::NextColumn();
      ImGui::Text("theta_cmd");
      ImGui::NextColumn();
      ImGui::Columns(1);
      ImGui::Separator();

      for (size_t i = 0; i < robot_system_ptr->joints.size(); ++i)
      {
        ImGui::Columns(5, "motion_display_column");
        ImGui::Text("joint %zu", i);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).data[mc::response][mc::x]);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).data[mc::response][mc::dx]);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).data[mc::response][mc::f_dis]);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->get_from_dict("theta_cmd"));
        ImGui::NextColumn();
        ImGui::Columns(1);
      }
    }

    static void control_mode_selection(robot_system *robot_system_ptr)
    {
      ImGui::Columns(mc::control_mode_size, "control mode");

      for (int i = 0; i < mc::control_mode_size; ++i)
      {
        const auto mode = static_cast<mc::control_mode>(i);
        std::string mode_name = mc::enum_helper::name(mode);
        if(ImGui::Button(mode_name.c_str(), ImVec2(-1.0f, 0.0f)))
        {
          robot_system_ptr->control_mode_request = static_cast<mc::control_mode>(i);
          std::cout << "mode [ " << mode_name << "] is seleted" << std::endl ;
        }
        ImGui::NextColumn();
      }
    }

#define x_res(n) robot_system_ptr->joints[(n)].data[mc::response][mc::x]
#define f_dis(n) robot_system_ptr->joints[(n)].data[mc::response][mc::f_dis]
    static void plot_state(robot_system *robot_system_ptr)
    {
      static std::vector<scrolling_buffer> sdata_vec(3);
      static float history = 2.0f;

      gui_time += ImGui::GetIO().DeltaTime;
      float t = gui_time;
      sdata_vec[0].add_point(t, static_cast<float>(x_res(0)));
      sdata_vec[1].add_point(t, static_cast<float>(f_dis(0)));
      sdata_vec[2].add_point(t, static_cast<float>(robot_system_ptr->get_from_dict("theta_cmd")));

      ImPlot::SetNextPlotLimitsX(t - history, t + history, ImGuiCond_Always);
      ImPlot::SetNextPlotLimitsY(-1, 1);
      if (sdata_vec[0].data.size() > 0 && ImPlot::BeginPlot("position / command", "time[sec]", "x[rad]", ImVec2(-1, 300)))
      {
        ImPlot::PlotLine("theta_res", &sdata_vec[0].data[0].x, &sdata_vec[0].data[0].y, sdata_vec[0].data.size(), sdata_vec[0].offset, 2*sizeof(float));
        if (sdata_vec[2].data.size() > 0)
          ImPlot::PlotLine("theta_cmd", &sdata_vec[2].data[0].x, &sdata_vec[2].data[0].y, sdata_vec[2].data.size(), sdata_vec[2].offset, 2*sizeof(float));
        ImPlot::EndPlot();
      }

      ImPlot::SetNextPlotLimitsX(t - history, t + history, ImGuiCond_Always);
      ImPlot::SetNextPlotLimitsY(-1, 1);
      if (sdata_vec[1].data.size() > 0 && ImPlot::BeginPlot("force", "time[sec]", "f[Nm]", ImVec2(-1, 200)))
      {
        ImPlot::PlotLine("f_dis", &sdata_vec[1].data[0].x, &sdata_vec[1].data[0].y, sdata_vec[1].data.size(), sdata_vec[1].offset, 2*sizeof(float));
        ImPlot::EndPlot();
      }
    }
#undef x_res
#undef f_dis

    static void finger_tracker_status(robot_system *robot_system_ptr)
    {
      static scrolling_buffer dist_buf;
      static float history = 5.0f;

      float t = gui_time;
      double distance_mm = robot_system_ptr->get_from_dict("distance_mm");
      double packet_count = robot_system_ptr->get_from_dict("packet_count");

      ImGui::Text("distance: %.1f mm", distance_mm);
      ImGui::Text("packets: %.0f", packet_count);
      ImGui::Text("recording: %s", robot_system_ptr->is_recording ? "YES" : "no");
      dist_buf.add_point(t, static_cast<float>(distance_mm));

      ImPlot::SetNextPlotLimitsX(t - history, t + history, ImGuiCond_Always);
      ImPlot::SetNextPlotLimitsY(0, 200);
      if (dist_buf.data.size() > 0 && ImPlot::BeginPlot("finger distance", "time[sec]", "dist[mm]", ImVec2(-1, 200)))
      {
        ImPlot::PlotLine("distance_mm", &dist_buf.data[0].x, &dist_buf.data[0].y, dist_buf.data.size(), dist_buf.offset, 2*sizeof(float));
        ImPlot::EndPlot();
      }
    }
  };
} // namespace mc
#endif //TELEOPHAND_GUI_WIDGET_H
