#ifndef MOTIONCONTOL_READER_H
#define MOTIONCONTOL_READER_H

#include <iostream>
#include <vector>
#include "robot_system.h"

template<typename T>
class reader
{
public:
  //method
  reader(size_t start_channel, size_t number_of_channel, robot_system *robot, mc::state read_state = mc::pulse_count):start_channel_(start_channel),
  number_of_channel_(number_of_channel),
  read_state_(read_state),
  read_data_ptr_(number_of_channel_)
  {
    register_read_data(robot);
    //open();
    device_id_++;
    std::cout << "reader constructor" << std::endl;
  }
  virtual int open() = 0;
  virtual int read() = 0;
  //virtual int read(size_t idx) = 0;
  virtual void reset() = 0;
  virtual void initialize() = 0;
  virtual void close() = 0;
  T *channel_ptr(size_t i){ return read_data_ptr_[i]; }
protected:
  size_t start_channel_;
  size_t number_of_channel_;
  mc::state read_state_;
private:
  static int device_id_;
  std::vector<T*> read_data_ptr_;

  // method
  void register_read_data(robot_system *robot)
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
      read_data_ptr_.at(i) = &(robot->joints.at(start_channel_ + i).data[mc::response][read_state_]);
    }
  }
};

template<typename T>
int reader<T>::device_id_ = 0;
#endif //MOTIONCONTOL_READER_H
