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

#ifndef CYBER_IO_POLL_HANDLER_H_
#define CYBER_IO_POLL_HANDLER_H_

#include <atomic>
#include <memory>

#include "cyber/croutine/croutine.h"
#include "cyber/io/poll_data.h"

namespace apollo {
namespace cyber {
namespace io {

class PollHandler {
 public:
  explicit PollHandler(int fd);
  virtual ~PollHandler() = default;

  bool Block(int timeout_ms, bool is_read);
  bool Unblock();

  int fd() const { return fd_; }
  void set_fd(int fd) { fd_ = fd; }

 private:
  bool Check(int timeout_ms);
  void Fill(int timeout_ms, bool is_read);
  void ResponseCallback(const PollResponse& rsp);

  int fd_;
  PollRequest request_;
  PollResponse response_;
  std::atomic<bool> is_read_;
  std::atomic<bool> is_blocking_;
  croutine::CRoutine* routine_;
};

}  // namespace io
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_IO_POLL_HANDLER_H_
