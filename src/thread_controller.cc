#include "thread_controller.h"
#include <mutex>
#include <condition_variable>
#include <atomic>

const int               MAX_NUMBER_OF_THREADS = 20;
std::mutex              mtx_vec[MAX_NUMBER_OF_THREADS] ;
std::condition_variable cv_vec[MAX_NUMBER_OF_THREADS] ;
bool                    is_ready_vec[MAX_NUMBER_OF_THREADS] = {false} ;// for spurious wakeup

// utility function for mutex
void notify_waiting_thread(int thread_id){
  {
    std::lock_guard<std::mutex> lock(mtx_vec[thread_id]);
    is_ready_vec[thread_id] = true;
  }
  cv_vec[thread_id].notify_one();
}

void wait_for_sample_time(int thread_id){
  std::unique_lock<std::mutex> uniq_lk(mtx_vec[thread_id]);
  cv_vec[thread_id].wait(uniq_lk, [thread_id]{return is_ready_vec[thread_id];});
  is_ready_vec[thread_id] = false;
}

void thread_controller::do_sample_time()
{
  static long long int tick(0);

  add_time_mesurement();
  while(!should_finish_all_){
#ifdef REALTIME
    timer_.wait_until_next_sample() ;
#endif
    tick++;
    for (auto &entry: frequency_to_thread_id)
      if (entry.first > 0 && tick%entry.first == 0)
        for (auto &id : entry.second)
          notify_waiting_thread(id);
  }
  for (auto &entry: frequency_to_thread_id)
      for (auto &id : entry.second)
        notify_waiting_thread(id);
}

void thread_controller::each_loop_of_realtime_task(mc::thread::thread_list thread_id)
{
  while(!should_finish_all_)
  {
    wait_for_sample_time(thread_id);
    system_controller_ptr_->set_timer_sampling_time(control_timer::get_micro_time(thread_id) - start_time_vec_.at(thread_id), thread_id);
    start_time_vec_.at(thread_id) = control_timer::get_micro_time(thread_id);
    system_controller_ptr_->tasks_[thread_id](thread_id_to_sampling_frequency[thread_id]);
    end_time_vec_.at(thread_id) = control_timer::get_micro_time(thread_id);
    system_controller_ptr_->set_task_takt_time(end_time_vec_.at(thread_id) - start_time_vec_.at(thread_id), thread_id);
  }
}
