#ifndef MOTIONCONTOL_WRITE_H
#define MOTIONCONTOL_WRITE_H

#include <iostream>
#include "robot_system.h"

template<typename T>
class writer
{
public:
  //method
  writer(size_t start_channel, size_t number_of_channel, robot_system *robot, mc::state write_state = mc::voltage)
  :start_channel_(start_channel),
  number_of_channel_(number_of_channel),
  write_state_(write_state),
  write_data_ptr_(number_of_channel_)
  {
    register_write_data(robot);
    device_id_++;
    std::cout << "writer constructor" << std::endl;
  }
  virtual int open() = 0;
  virtual int write(){write_buf(); flush(); return number_of_channel_;}
  //virtual int write(size_t idx) = 0;
  virtual int write_buf() = 0;
  virtual int flush() = 0;
  virtual void reset() = 0;
  virtual void zero() = 0;
  virtual void initialize() = 0;
  virtual void close() = 0;
  T *channel_ptr(size_t i){ return write_data_ptr_[i]; }
protected:
  size_t start_channel_;
  size_t number_of_channel_;
  mc::state write_state_;
private:
  static int device_id_;
  std::vector<T*> write_data_ptr_;

  // method
  void register_write_data(robot_system *robot)
  {
    if (robot->joints.size() < start_channel_ + number_of_channel_)
    {
      std::cerr << "Warning:" << std::endl;
      std::cerr << "\t You try to register unexist joint." << std::endl;
      std::cerr << "\t number_of_channel is too large." << std::endl;
      std::cerr << "\t device id: " << device_id_ << std::endl;
      exit(1);
    }
    for (size_t i = 0; i < number_of_channel_ ; ++i)
    {
      write_data_ptr_.at(i) = &(robot->joints.at(start_channel_ + i).data[mc::output][write_state_]);
    }
  }
};
template<typename T>
int writer<T>::device_id_ = 0;
#endif //MOTIONCONTOL_WRITE_H

