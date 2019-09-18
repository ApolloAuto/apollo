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

#include "cyber/io/poll_handler.h"
#include "cyber/common/log.h"
#include "cyber/io/poller.h"
#include "cyber/scheduler/scheduler_factory.h"

namespace apollo {
namespace cyber {
namespace io {

using croutine::CRoutine;
using croutine::RoutineState;

PollHandler::PollHandler(int fd)
    : fd_(fd), is_read_(false), is_blocking_(false), routine_(nullptr) {}

bool PollHandler::Block(int timeout_ms, bool is_read) {
  if (!Check(timeout_ms)) {
    return false;
  }

  if (is_blocking_.exchange(true)) {
    AINFO << "poll handler is blocking.";
    return false;
  }

  Fill(timeout_ms, is_read);
  if (!Poller::Instance()->Register(request_)) {
    is_blocking_.store(false);
    return false;
  }

  routine_->Yield(RoutineState::IO_WAIT);

  bool result = false;
  uint32_t target_events = is_read ? EPOLLIN : EPOLLOUT;
  if (response_.events & target_events) {
    result = true;
  }
  is_blocking_.store(false);

  return result;
}

bool PollHandler::Unblock() {
  is_blocking_.store(false);
  return Poller::Instance()->Unregister(request_);
}

bool PollHandler::Check(int timeout_ms) {
  if (timeout_ms == 0) {
    AINFO << "timeout[" << timeout_ms
          << "] must be larger than zero or less than zero.";
    return false;
  }

  if (fd_ < 0) {
    AERROR << "invalid fd[" << fd_ << "]";
    return false;
  }

  routine_ = CRoutine::GetCurrentRoutine();
  if (routine_ == nullptr) {
    AERROR << "routine nullptr, please use IO in routine context.";
    return false;
  }

  return true;
}

void PollHandler::Fill(int timeout_ms, bool is_read) {
  is_read_.store(is_read);

  request_.fd = fd_;
  request_.events = EPOLLET | EPOLLONESHOT;
  if (is_read) {
    request_.events |= EPOLLIN;
  } else {
    request_.events |= EPOLLOUT;
  }
  request_.timeout_ms = timeout_ms;
  request_.callback =
      std::bind(&PollHandler::ResponseCallback, this, std::placeholders::_1);
}

void PollHandler::ResponseCallback(const PollResponse& rsp) {
  if (!is_blocking_.load() || routine_ == nullptr) {
    return;
  }

  response_ = rsp;

  if (routine_->state() == RoutineState::IO_WAIT) {
    scheduler::Instance()->NotifyTask(routine_->id());
  }
}

}  // namespace io
}  // namespace cyber
}  // namespace apollo
