//
// Created by xiaobai on 2021/07/20.
//

#ifndef MOTIONCONTROL_CONTROL_TIMER_H
# define MOTIONCONTROL_CONTROL_TIMER_H
#include <ctime>
#include <utility>
#include <vector>

class control_timer
{
public:
                        control_timer(long long int sample_frequency);
  void                  wait_until_next_sample();
  void                  set_initial_time();
  void                  set_time(long long int&) ;
  static long long int  get_micro_time();
  static long long int  get_micro_time(int id);
  static long long int  get_micro_time_from_timespec(timespec &t);
  static long long int  get_micro_time_from_timespec(clockid_t kind, timespec &t);
  void                  setNanoToTick(double nanoToTick);
  static void           add_timer();
  double                get_sample_frequency(){ return sample_frequency_; }
private:
  long long int                 sample_frequency_;
  long long int                 initial_micro_time_ ;
  timespec                      timespec_;
  long long int                 tick_;//time cnt which counts at the sampling frequency
  long long int                 last_tick_;
  double                        nano_to_tick_;
  static std::vector<timespec>  timespec_vec_;
};

# endif //MOTIONCONTROL_CONTROL_TIMER_H
