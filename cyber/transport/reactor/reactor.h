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

#ifndef CYBER_TRANSPORT_REACTOR_REACTOR_H_
#define CYBER_TRANSPORT_REACTOR_REACTOR_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "cyber/common/macros.h"

namespace apollo {
namespace cyber {
namespace transport {

using Handle = int;
using EventsType = uint32_t;
const int INVALID_FD = -1;
const int MAX_EVENTS = 1024;

class Reactor {
 public:
  using EventHandler = std::function<void(const EventsType&)>;

  virtual ~Reactor();

  void Start();
  void Shutdown();

  bool Add(const Handle& handle, const EventsType& events,
           const EventHandler& event_handler);
  bool Modify(const Handle& handle, const EventsType& events);
  bool Delete(const Handle& handle);

 private:
  struct HandlerInfo {
    Handle handle;
    EventsType events;
    EventHandler event_handler;
  };
  using HandlerMap = std::unordered_map<Handle, HandlerInfo>;

  void ThreadFunc();
  void Poll(int timeout);
  void NotifyToChangeHandler();
  void OnHandlerChanged(const EventsType& events);
  void DisposeChangedHandler();
  bool CreatePipe();

  std::atomic<bool> shutdown_;
  int epoll_fd_;

  HandlerMap handlers_;
  std::mutex handlers_mutex_;

  bool changed_;
  HandlerMap handlers_to_add_;
  HandlerMap handlers_to_mod_;
  HandlerMap handlers_to_del_;
  std::mutex change_mutex_;

  int pipe_fd_[2];
  std::mutex pipe_mutex_;

  std::thread thread_;

  DECLARE_SINGLETON(Reactor)
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_REACTOR_REACTOR_H_
