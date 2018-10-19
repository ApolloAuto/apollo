/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/transport/reactor/reactor.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/epoll.h>
#include <unistd.h>

#include "cyber/common/log.h"
#include "cyber/transport/common/syscall_wrapper.h"

namespace apollo {
namespace cyber {
namespace transport {

Reactor::Reactor() : shutdown_(true), epoll_fd_(INVALID_FD), changed_(false) {
  pipe_fd_[0] = INVALID_FD;
  pipe_fd_[1] = INVALID_FD;
}

Reactor::~Reactor() { Shutdown(); }

void Reactor::Start() {
  if (!shutdown_) {
    return;
  }

  epoll_fd_ = epoll_create(MAX_EVENTS);
  RETURN_IF(epoll_fd_ < 0);
  if (!CreatePipe()) {
    CloseAndReset(&epoll_fd_);
    CloseAndReset(&pipe_fd_[0]);
    CloseAndReset(&pipe_fd_[1]);
    return;
  }

  shutdown_ = false;
  thread_ = std::thread(&Reactor::ThreadFunc, this);
}

void Reactor::Shutdown() {
  // avoid shutdown twice
  if (shutdown_.exchange(true)) {
    return;
  }

  if (thread_.joinable()) {
    thread_.join();
  }

  CloseAndReset(&epoll_fd_);
  CloseAndReset(&pipe_fd_[0]);
  CloseAndReset(&pipe_fd_[1]);

  handlers_.clear();
  handlers_to_add_.clear();
  handlers_to_mod_.clear();
  handlers_to_del_.clear();
}

bool Reactor::Add(const Handle& handle, const EventsType& events,
                  const EventHandler& event_handler) {
  HandlerInfo info;
  info.handle = handle;
  info.events = events;
  info.event_handler = event_handler;

  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto itr = handlers_.find(handle);
    if (itr != handlers_.end()) {
      return false;
    }
    handlers_[handle] = info;
  }

  {
    std::lock_guard<std::mutex> lock(change_mutex_);
    handlers_to_add_[handle] = info;
    changed_ = true;
  }

  NotifyToChangeHandler();

  return true;
}

bool Reactor::Modify(const Handle& handle, const EventsType& events) {
  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto itr = handlers_.find(handle);
    if (itr == handlers_.end()) {
      return false;
    }
    handlers_[handle].events = events;
  }

  HandlerInfo info;
  info.handle = handle;
  info.events = events;

  {
    std::lock_guard<std::mutex> lock(change_mutex_);
    handlers_to_mod_[handle] = info;
    changed_ = true;
  }

  NotifyToChangeHandler();

  return true;
}

bool Reactor::Delete(const Handle& handle) {
  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto itr = handlers_.find(handle);
    if (itr == handlers_.end()) {
      return false;
    }
    handlers_.erase(itr);
  }

  HandlerInfo info;
  info.handle = handle;
  info.events = 0;

  {
    std::lock_guard<std::mutex> lock(change_mutex_);
    handlers_to_del_[handle] = info;
    changed_ = true;
  }

  NotifyToChangeHandler();

  return true;
}

void Reactor::ThreadFunc() {
  /* block all signals in this thread */
  sigset_t signal_set;
  sigfillset(&signal_set);
  pthread_sigmask(SIG_BLOCK, &signal_set, nullptr);

  while (!shutdown_) {
    Poll(100);
  }
}

void Reactor::Poll(int timeout) {
  DisposeChangedHandler();

  epoll_event evt[MAX_EVENTS];
  int ready_num = epoll_wait(epoll_fd_, evt, MAX_EVENTS, timeout);
  if (ready_num < 0) {
    if (errno != EINTR) {
      // print error log
    }
    return;
  }

  for (int i = 0; i < ready_num; ++i) {
    Handle handle = evt[i].data.fd;
    EventsType events = evt[i].events;
    EventHandler event_handler;

    {
      std::lock_guard<std::mutex> lock(handlers_mutex_);
      auto itr = handlers_.find(handle);
      if (itr != handlers_.end()) {
        event_handler = itr->second.event_handler;
      }
    }

    // we may opt this by half-sync half-async
    if (event_handler) {
      event_handler(events);
    }
  }
}

void Reactor::NotifyToChangeHandler() {
  std::unique_lock<std::mutex> lock(pipe_mutex_, std::try_to_lock);
  RETURN_IF(!lock.owns_lock());

  char msg = 'B';
  if (write(pipe_fd_[1], &msg, 1) < 0) {
    if (errno == EBADF) {
      AWARN << "you should start reactor firstly.";
    }
  }
}

void Reactor::OnHandlerChanged(const EventsType& events) {
  char msg = '\0';
  while (read(pipe_fd_[0], &msg, 1) > 0) {
    // do nothing
  }
}

void Reactor::DisposeChangedHandler() {
  if (!changed_) {
    return;
  }

  HandlerMap add;
  HandlerMap mod;
  HandlerMap del;

  {
    std::lock_guard<std::mutex> lock(change_mutex_);
    add.swap(handlers_to_add_);
    mod.swap(handlers_to_mod_);
    del.swap(handlers_to_del_);
    changed_ = false;
  }

  if (!add.empty()) {
    for (auto& item : add) {
      epoll_event evt;
      evt.events = item.second.events;
      evt.data.fd = item.second.handle;
      if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, item.second.handle, &evt) != 0) {
        // print error log
      }
    }
  }

  if (!mod.empty()) {
    for (auto& item : mod) {
      epoll_event evt;
      evt.events = item.second.events;
      evt.data.fd = item.second.handle;
      if (epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, item.second.handle, &evt) != 0) {
        // print error log
      }
    }
  }

  if (!del.empty()) {
    for (auto& item : del) {
      epoll_event evt;
      if (epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, item.second.handle, &evt) != 0) {
        // print error log
      }
    }
  }
}

bool Reactor::CreatePipe() {
  RETURN_VAL_IF(pipe(pipe_fd_) != 0, false);
  RETURN_VAL_IF(!SetNonBlocking(pipe_fd_[0]), false);
  RETURN_VAL_IF(!SetNonBlocking(pipe_fd_[1]), false);

  return Add(pipe_fd_[0], EPOLLIN, std::bind(&Reactor::OnHandlerChanged, this,
                                             std::placeholders::_1));
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
