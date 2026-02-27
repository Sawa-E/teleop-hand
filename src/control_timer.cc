//
// Created by xiaobai on 2021/07/20.
//

#include <iostream>
#include "control_timer.h"

const long long int ORDER_OF_GIGA = 1000000000;
const double        SEC_TO_MICRO  = 1000000.0;
const double        NANO_TO_MICRO = 0.001;
std::vector<timespec> control_timer::timespec_vec_;

// utility function
long long int control_timer::get_micro_time_from_timespec(timespec& t)
{
  clock_gettime(CLOCK_MONOTONIC_RAW, &t);
  return static_cast<long long int>(t.tv_sec * SEC_TO_MICRO + t.tv_nsec * NANO_TO_MICRO);
}

long long int control_timer::get_micro_time_from_timespec(clockid_t kind, timespec& t)
{
  clock_gettime(kind, &t);
  return static_cast<long long int>(t.tv_sec * SEC_TO_MICRO + t.tv_nsec * NANO_TO_MICRO);
}

// class definition
control_timer::control_timer(long long int sample_frequency)
:sample_frequency_(sample_frequency)
{
  nano_to_tick_ = static_cast<double>(sample_frequency_) / ORDER_OF_GIGA;
  std::cout << "control timer constructor" << std::endl;
  std::cout << "\tsample freq: " << sample_frequency_ << std::endl;
  std::cout << "\tnano to tick: " << nano_to_tick_ << std::endl;
}

void control_timer::wait_until_next_sample()
{
  while (1)
  {
    clock_gettime(CLOCK_MONOTONIC_RAW, &timespec_);
    tick_ = timespec_.tv_nsec * nano_to_tick_;
    if((tick_) % 1 == 0 && last_tick_ != tick_)
    {
      last_tick_ = tick_;
      break;
    }
  }
}

void control_timer::set_initial_time()
{
  clock_gettime(CLOCK_REALTIME, &timespec_);
  initial_micro_time_ = control_timer::get_micro_time_from_timespec(timespec_);
  std::cout << "init time: " << initial_micro_time_ << std::endl ;
}

void control_timer::set_time(long long int & absolute_micro_time)
{
  absolute_micro_time = control_timer::get_micro_time_from_timespec(timespec_)  + initial_micro_time_ ;
}

void control_timer::setNanoToTick(double nano_to_tick) {
  nano_to_tick_ = nano_to_tick;
}

long long int control_timer::get_micro_time()
{
  timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return static_cast<long long int>(t.tv_sec * SEC_TO_MICRO + t.tv_nsec * NANO_TO_MICRO);
}

long long int control_timer::get_micro_time(int id)
{
  // not implemented
  return control_timer::get_micro_time_from_timespec(CLOCK_MONOTONIC_RAW, control_timer::timespec_vec_.at(id));
}

void control_timer::add_timer()
{
  control_timer::timespec_vec_.push_back(timespec());
  std::cout << "timespece is added" << std::endl;
}
