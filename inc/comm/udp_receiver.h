#ifndef TELEOPHAND_UDP_RECEIVER_H
#define TELEOPHAND_UDP_RECEIVER_H

#include <mutex>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "comm/finger_data.h"
#include "control_timer.h"

namespace mc {

class udp_receiver
{
public:
  udp_receiver() : socket_fd_(-1), packet_count_(0), last_receive_time_(0), valid_(false), latest_data_{} {}

  int open(int port)
  {
    if (socket_fd_.load() >= 0) close(); // prevent socket leak on re-open
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
      std::cerr << "[udp_receiver] socket creation failed" << std::endl;
      return -1;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      std::cerr << "[udp_receiver] bind failed on port " << port << std::endl;
      ::close(fd);
      return -1;
    }

    socket_fd_.store(fd);
    std::cout << "[udp_receiver] listening on port " << port << std::endl;
    return 0;
  }

  void receive()
  {
    int fd = socket_fd_.load();
    if (fd < 0) return;

    finger_data buf;
    ssize_t n = recvfrom(fd, &buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr);
    // validate: correct size and non-negative distance (ADR 002: -1.0 means detection failure)
    if (n == static_cast<ssize_t>(sizeof(finger_data)) && buf.distance_mm >= 0.0f)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_data_ = buf;
      valid_ = true;
      packet_count_++;
      last_receive_time_ = control_timer::get_micro_time();
    }
  }

  // returns {is_valid, data} as a single atomic operation under mutex
  std::pair<bool, finger_data> get_latest_data()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {valid_, latest_data_};
  }

  uint64_t get_packet_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return packet_count_;
  }

  long long int get_last_receive_time()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_receive_time_;
  }

  void close()
  {
    int fd = socket_fd_.exchange(-1);
    if (fd >= 0)
    {
      ::close(fd);
    }
  }

  ~udp_receiver() { close(); }

private:
  std::atomic<int> socket_fd_;
  std::mutex mutex_;
  finger_data latest_data_;
  uint64_t packet_count_;
  long long int last_receive_time_;
  bool valid_;
};

} // namespace mc
#endif //TELEOPHAND_UDP_RECEIVER_H
