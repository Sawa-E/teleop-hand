#ifndef MOTIONCONTROL_CONTEC_COUNTER_H
#define MOTIONCONTROL_CONTEC_COUNTER_H
#include <sys/io.h>
#include "reader.h"
#include "json_helper.h"
#include "pci_helper.h"

struct contec_cnt_device_set{
  unsigned int base0;
  unsigned int base1;
};

template<typename T>
class contec_counter : public reader<T>
{
public:
  const int SUCCESS = 0;
  const int FAIL = 1;

  // method
  contec_counter(size_t start_channel, size_t number_of_channel, robot_system *robot, const char *config_file_name)
  :reader<T>(start_channel, number_of_channel, robot),
  config_file_name_(config_file_name)
  {
    set_pulse_per_rotatation(robot);
    open();
    std::cout << "contec conter constructor" << std::endl;
  }

  int open()
  {
    boost::property_tree::ptree pt;
    read_json(config_file_name_.c_str(), pt);
    unsigned int vendor_id = static_cast<unsigned int>(strtol(json_helper<std::string>::get_value_from_json("vendor_id", pt).c_str(), NULL, 16));
    unsigned int device_id = static_cast<unsigned int>(strtol(json_helper<std::string>::get_value_from_json("device_id", pt).c_str(), NULL, 16));
    int board_order = json_helper<int>::get_value_from_json("board_order", pt);
    counter_pci_.base0 = pci_helper::get_io_port_from_vendor_and_device_id(vendor_id, device_id, board_order);
    std::cout << "[reader] vendor_id: " << vendor_id << std::endl;
    std::cout << "[reader] device_id: " << device_id << std::endl;
    std::cout << "[reader] io port: " << counter_pci_.base0 << std::endl;
    initialize();
    return SUCCESS;
  }

  int read()
  {
    for (size_t i = 0; i < reader<T>::number_of_channel_; ++i)
    {
      read(i);
    }
    return SUCCESS;
  }

  int read(size_t ch)
  {
    int count;

    switch(ch){
      case 0:
        outw(0x00ff, counter_pci_.base0 + 0x02);
        outw(0x0000, counter_pci_.base0 + 0x00);
        break;
      case 1:
        outw(0x0001, counter_pci_.base0 + 0x00);
        break;
      case 2:
        outw(0x0002, counter_pci_.base0 + 0x00);
        break;
      case 3:
        outw(0x0003, counter_pci_.base0 + 0x00);
        break;
      case 4:
        outw(0x0004, counter_pci_.base0 + 0x00);
        break;
      case 5:
        outw(0x0005, counter_pci_.base0 + 0x00);
        break;
      case 6:
        outw(0x0006, counter_pci_.base0 + 0x00);
        break;
      case 7:
        outw(0x0007, counter_pci_.base0 + 0x00);
        break;
    }

    count = inl(counter_pci_.base0+0x00);

    *(reader<T>::channel_ptr(ch)) = static_cast<double>(count) * inverse_pulse_per_rotation_.at(ch);
    return reader<T>::number_of_channel_;
  }

  void reset()
  {
    outw(0x0008, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x0009, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x000a, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x000b, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x000c, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x000d, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x000e, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x000f, counter_pci_.base0 + 0x08);
    outl(0x00000022, counter_pci_.base0 + 0x0c);
    outw(0x0010, counter_pci_.base0 + 0x08);
    outl(0x00000000, counter_pci_.base0 + 0x0c);

    outw(0x0030, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0031, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0032, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0033, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0034, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0035, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0036, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x0037, counter_pci_.base0 + 0x08);
    outl(0x00000001, counter_pci_.base0 + 0x0c);
    outw(0x003c, counter_pci_.base0 + 0x08);
    outl(0x000000ff, counter_pci_.base0 + 0x0c);

    std::cout << "Counter Start" << std::endl;
    outw(0x00ff, counter_pci_.base0 + 0x04);
  }

  void initialize()
  {
    std::cout << "started contec counter board" << std::endl;
    iopl(3);// need root permission
    reset();
  }

  void close()
  {
    std::cout << "contec counter is closed" << std::endl;
    reset();
  }

private:
  std::vector<T> inverse_pulse_per_rotation_;
  contec_cnt_device_set counter_pci_;
  std::string config_file_name_;
  std::string vendor_id_;
  std::string device_id_;

  // method
  void set_pulse_per_rotatation(robot_system *robot)
  {
    inverse_pulse_per_rotation_.clear();
    for (size_t i = reader<T>::start_channel_; i < reader<T>::number_of_channel_ + reader<T>::start_channel_; ++i)
    {
      T pulse_per_rotation = robot->joints[i].parameter[mc::pulse_per_rotation] * robot->joints[i].parameter[mc::multiplication] * robot->joints[i].parameter[mc::gear_ratio];
      if (robot->joints[i].parameter[mc::rotate_or_linear] < 0.5)
        pulse_per_rotation /= (2.0 * M_PI);
      inverse_pulse_per_rotation_.push_back(1.0 / pulse_per_rotation);
    }
  }
};
#endif //MOTIONCONTROL_CONTEC_COUNTER_H
