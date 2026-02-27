#ifndef MOTIONCONTROL_THREAD_CONTROLLER_H
#define MOTIONCONTROL_THREAD_CONTROLLER_H
#include <iostream>
#include <vector>
#include <thread>
#include <unordered_map>
#include "control_timer.h"
#include "system_controller.h"

struct thread_config
{
  std::string thread_name;
  int sampling_frequency;
  bool is_single_loop;
};

class thread_controller
{
private:
  std::vector<std::thread> threads_;
  std::thread timer_thread_;
  bool should_finish_all_;
  control_timer timer_;
  std::vector<long long int> thread_id_to_sampling_frequency;
  std::unordered_map<long long int, std::vector<int>> frequency_to_thread_id;
  system_controller *system_controller_ptr_;
  std::vector<long long int> start_time_vec_;
  std::vector<long long int> end_time_vec_;
public:
  thread_controller(long long int sample_frequency, system_controller* system_controller_ptr)
  :should_finish_all_(false),
  timer_(control_timer(sample_frequency)),
  system_controller_ptr_(system_controller_ptr)
  {
    std::cout << "constructor thread controller" << std::endl;
  }

  thread_controller(control_timer timer, system_controller *system_controller_ptr)
  :should_finish_all_(false),
  timer_(timer),
  system_controller_ptr_(system_controller_ptr)
  {
    std::cout << "constructor thread controller" << std::endl;
  }

  ~thread_controller(){join(); system_controller_ptr_->close();}

  void start_timer()
  {
    timer_thread_ = std::thread(&thread_controller::do_sample_time, this);
  }

  int thread_registeration()
  {
    for (int i = 0 ; i < mc::thread::thread_list_size; ++i)
    {
      add_time_mesurement();
      auto thread_id = static_cast<mc::thread::thread_list>(i);

      std::string thread_name = mc::enum_helper::name<mc::thread::thread_list>(thread_id);
      auto config = get_config(thread_name.c_str());

      if (config.is_single_loop)
      {
        int frequency = config.sampling_frequency;
        if (frequency > 0)
          threads_.push_back(std::thread(&thread_controller::each_loop_of_realtime_task, this, thread_id));
        else
          threads_.push_back(std::thread(&thread_controller::each_loop_of_task, this, thread_id));
        frequency_to_thread_id[frequency].push_back(thread_id);
        thread_id_to_sampling_frequency[thread_id] = timer_.get_sample_frequency() / frequency;
      }
      else
        threads_.push_back(std::thread(&thread_controller::all_loop_of_task, this, thread_id));
    }
    return 0;
  }
  void join()
  {
    std::cout << "joining threads..." << std::endl;
    for (auto &thread : threads_)
      thread.join();
    timer_thread_.join();
    std::cout << "all threads are joined successfully" << std::endl;
  }

  void join(int idx)
  {
    threads_.at(idx).join();
  }

private:
  void each_loop_of_realtime_task(mc::thread::thread_list thread_id);

  void each_loop_of_task(mc::thread::thread_list thread_id)
  {
    while(!should_finish_all_)
    {
      start_time_vec_.at(thread_id) = control_timer::get_micro_time(thread_id);
      system_controller_ptr_->set_timer_sampling_time(start_time_vec_.at(thread_id) - end_time_vec_.at(thread_id), thread_id);
      system_controller_ptr_->tasks_[thread_id](0);
      end_time_vec_.at(thread_id) = control_timer::get_micro_time(thread_id);
      system_controller_ptr_->set_task_takt_time(end_time_vec_.at(thread_id) - start_time_vec_.at(thread_id), thread_id);
    }
  }

  void all_loop_of_task(mc::thread::thread_list thread_id)
  {
      if ((system_controller_ptr_->tasks_[thread_id](0)) == system_controller::FINISH)
        should_finish_all_ = true;
  }

  void print_config(thread_config &config)
  {
    std::cout << "thread name: [" << config.thread_name << "]" <<  std::endl;
    std::cout << "\tsampling frequency: " << config.sampling_frequency << "Hz per timer count." << std::endl;
    std::cout << "\tis single loop[1: yes, 0: no]: " << config.is_single_loop << std::endl;
  }

  thread_config get_config(const char *thread_name)
  {
    thread_config ret;
    const char *file_name_ = "../config/thread_config.json";
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(file_name_, pt);

    auto child_opt = pt.get_child_optional(thread_name);
    if (child_opt)
    {
      auto node = child_opt.get();
      auto sampling_frequency_node = node.get_optional<int>("sampling_frequency");
      auto is_single_loop_node = node.get_optional<bool>("is_single_loop");

      if (sampling_frequency_node && is_single_loop_node)
      {
        ret.thread_name = thread_name;
        ret.sampling_frequency = sampling_frequency_node.get();
        ret.is_single_loop = is_single_loop_node.get();
        print_config(ret);
        return ret;
      }
    }
    std::cerr << "[Warning:]" << std::endl;
    std::cerr << "\t" << thread_name << " is not exist in " << file_name_ << std::endl;
    if (std::string(thread_name) == "default")
    {
      std::cerr << "[Error:]" << std::endl;
      std::cerr << "\tno default config." << std::endl;
      exit(1);
    }
    return get_default();
  }

  thread_config get_default()
  {
    std::cerr << "[Warning:]" << std::endl;
    std::cerr << "\tset default config." << std::endl;
    auto ret = get_config("default");
    print_config(ret);
    return ret;
  }

  void add_time_mesurement()
  {
    system_controller_ptr_->add_timer_sampling_time();
    system_controller_ptr_->add_task_takt_time();
    control_timer::add_timer();
    start_time_vec_.push_back(0);
    end_time_vec_.push_back(0);
    thread_id_to_sampling_frequency.push_back(0);
  }

  void do_sample_time();
};
#endif //MOTIONCONTROL_THREAD_CONTROLLER_H
