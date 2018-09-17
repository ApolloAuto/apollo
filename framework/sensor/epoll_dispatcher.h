
#ifndef SENSOR_EPOLL_DISPATCHER_H_
#define SENSOR_EPOLL_DISPATCHER_H_

#include <sys/epoll.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <unordered_map>

#include "cybertron/common/macros.h"

namespace apollo {
namespace sensor {

static constexpr int EPOLL_SIZE = 1024 * 1024;
static constexpr int EPOLL_EVENT_SIZE = 32;

using Handler_callback = std::function<void(const epoll_event&)>;

// Dispatch edge-triggered events of file descriptors to consumers
class EpollDispatcher {
public:
  virtual ~EpollDispatcher();
  virtual bool Start();
  bool Running() const;
  void Stop();
  void Join();
  bool AddHandler(int fd, unsigned int event, Handler_callback handler);
  bool RemoveHandler(int fd);

private:
  void Run();
  int epfd_;
  volatile bool stop_;
  std::thread run_thread_;
  int wakeup_fds_[2];
  std::unordered_map<int, Handler_callback> handler_map_;

  DECLARE_SINGLETON(EpollDispatcher)
};

} // namespace drivers
} // namespace apollo

#endif //CYBERTRON_DRIVERS_EPOLL_DISPATCHER_H
