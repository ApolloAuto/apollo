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

#ifndef CYBER_IO_POLLER_H_
#define CYBER_IO_POLLER_H_

#include <atomic>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/common/macros.h"
#include "cyber/io/poll_data.h"

namespace apollo {
namespace cyber {
namespace io {

class Poller {
 public:
  using RequestPtr = std::shared_ptr<PollRequest>;
  using RequestMap = std::unordered_map<int, RequestPtr>;
  using CtrlParamMap = std::unordered_map<int, PollCtrlParam>;

  virtual ~Poller();

  void Shutdown();

  bool Register(const PollRequest& req);
  bool Unregister(const PollRequest& req);

 private:
  bool Init();
  void Clear();
  void Poll(int timeout_ms);
  void ThreadFunc();
  void HandleChanges();
  int GetTimeoutMs();
  void Notify();

  int epoll_fd_ = -1;
  std::thread thread_;
  std::atomic<bool> is_shutdown_ = {true};

  int pipe_fd_[2] = {-1, -1};
  std::mutex pipe_mutex_;

  RequestMap requests_;
  CtrlParamMap ctrl_params_;
  base::AtomicRWLock poll_data_lock_;

  const int kPollSize = 32;
  const int kPollTimeoutMs = 100;

  DECLARE_SINGLETON(Poller)
};

}  // namespace io
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_IO_POLLER_H_
