
#include  <sys/socket.h>
#include <sys/epoll.h>

#include "cybertron/common/log.h"
#include "sensor/epoll_dispatcher.h"

namespace apollo {
namespace sensor {

EpollDispatcher::EpollDispatcher()
    : epfd_(-1)
    , stop_(false)
{
  epfd_ = epoll_create(EPOLL_SIZE);
  if (epfd_ < 0) {
      AERROR << "Fail to create epoll";
      return;
  }

  wakeup_fds_[0] = -1;
  wakeup_fds_[1] = -1;
  if (pipe(wakeup_fds_) != 0) {
      AERROR << "Fail to create pipe";
      return;
  }
}

EpollDispatcher::~EpollDispatcher() {
  Stop();
  Join();
  if (epfd_ >= 0) {
      close(epfd_);
      epfd_ = -1;
  }
  if (wakeup_fds_[0] > 0) {
      close(wakeup_fds_[0]);
      close(wakeup_fds_[1]);
  }
}

bool EpollDispatcher::Start() {
  if (epfd_ < 0) {
      AERROR << "epoll was not created";
      return false;
  }
  
  run_thread_ = std::thread(std::bind(&EpollDispatcher::Run, this));  
  return true;
}

bool EpollDispatcher::Running() const {
  return !stop_  && epfd_ >= 0;
}

void EpollDispatcher::Stop() {
  stop_ = true;

  if (epfd_ >= 0) {
      epoll_event evt = { EPOLLOUT,  { NULL } };
      epoll_ctl(epfd_, EPOLL_CTL_ADD, wakeup_fds_[1], &evt);
  }
}

void EpollDispatcher::Join() {
  run_thread_.join();
}

bool EpollDispatcher::AddHandler(int fd, unsigned int event, Handler_callback handler) {
  
  AINFO << "add fd:" << fd;
  if (epfd_ < 0) {
      errno = EINVAL;
      return false;
  }
  epoll_event evt;
  evt.events = event;
  evt.data.fd = fd;
  if (epoll_ctl(epfd_, EPOLL_CTL_ADD, fd, &evt) < 0) {
    AERROR << "add handler failed for fd:" << fd;
    return false;
  }
  //one fd one handler now, and we add handler one by one
  auto result = handler_map_.find(fd);
  if (result == handler_map_.end()) {
    handler_map_[fd] = handler;
    AINFO << "add fd:" << fd;
    return true;
  } else {
    AINFO << "fd already has handler:" << fd;
    return false; 
  }
}

bool EpollDispatcher::RemoveHandler(int fd) {
  if (fd < 0) {
      return false;
  }
  if (epoll_ctl(epfd_, EPOLL_CTL_DEL, fd, NULL) < 0) {
      AERROR << "Fail to remove fd=" << fd << " from epfd=" << epfd_;
      return false;
  }
  //more check
  handler_map_.erase(fd);
  return true;
}

void EpollDispatcher::Run() {
  epoll_event e[EPOLL_EVENT_SIZE];
  while (!stop_) {
    const int n = epoll_wait(epfd_, e, EPOLL_EVENT_SIZE, -1);
    if (stop_) {
      break;
    }
    if (n < 0) {
      if (EINTR == errno) {
        // We've checked _stop, no wake-up will be missed.
        continue;
      }
      AERROR << "Fail to epoll_wait epfd=" << epfd_;
      break;
    }
    for (int i = 0; i < n; ++i) {
      int fd = e[i].data.fd;
      //we sure hanlder fd is exist
      handler_map_[fd](e[i]);
    }
  }
}

} // namespace drivers
} // namespace apollo 
