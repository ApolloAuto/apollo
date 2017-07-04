/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/can_comm/can_receiver.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include "modules/canbus/common/canbus_consts.h"
#include "modules/common/log.h"

namespace apollo {
namespace canbus {

using ::apollo::common::ErrorCode;

ErrorCode CanReceiver::Init(CanClient* can_client, MessageManager* pt_manager,
                            bool enable_log) {
  can_client_ = can_client;
  pt_manager_ = pt_manager;
  enable_log_ = enable_log;
  if (can_client_ == nullptr) {
    AERROR << "Invalid can client.";
    return ErrorCode::CANBUS_ERROR;
  }
  if (pt_manager_ == nullptr) {
    AERROR << "Invalid protocol manager.";
    return ErrorCode::CANBUS_ERROR;
  }
  is_init_ = true;
  return ErrorCode::OK;
}

void CanReceiver::RecvThreadFunc() {
  AINFO << "Can client receiver thread starts.";
  CHECK_NOTNULL(can_client_);
  CHECK_NOTNULL(pt_manager_);

  int32_t receive_error_count = 0;
  int32_t receive_none_count = 0;
  const int32_t ERROR_COUNT_MAX = 10;
  std::chrono::duration<double, std::micro> default_period{10 * 1000};

  while (IsRunning()) {
    std::vector<CanFrame> buf;
    int32_t length = MAX_CAN_RECV_FRAME_LEN;
    if (can_client_->Receive(&buf, &length) != ErrorCode::OK) {
      LOG_IF_EVERY_N(ERROR, receive_error_count++ > ERROR_COUNT_MAX,
                     ERROR_COUNT_MAX)
          << "Received " << receive_error_count << " error messages.";
      std::this_thread::sleep_for(default_period);
      continue;
    }
    receive_error_count = 0;

    if (buf.size() != static_cast<size_t>(length)) {
      AERROR << "Receiver buf size[" << buf.size()
             << "] does not match can_client returned length[" << length
             << "].";
    }

    if (length == 0) {
      LOG_IF_EVERY_N(ERROR, receive_none_count++ > ERROR_COUNT_MAX,
                     ERROR_COUNT_MAX)
          << "Received " << receive_none_count << " empty messages.";
      std::this_thread::sleep_for(default_period);
      continue;
    }
    receive_none_count = 0;

    for (int32_t i = 0; i < length; ++i) {
      uint8_t len = buf.at(i).len;
      uint32_t uid = buf.at(i).id;
      uint8_t* data = buf.at(i).data;
      struct timeval timestamp;
      timestamp.tv_sec = buf.at(i).timestamp.tv_sec;
      timestamp.tv_usec = buf.at(i).timestamp.tv_usec;
      pt_manager_->Parse(uid, data, len, timestamp);
      if (enable_log_) {
        ADEBUG << "recv_can_frame#" << buf.at(i).CanFrameString();
      }
    }
    std::this_thread::yield();
  }
  AINFO << "Can client receiver thread stopped.";
}

bool CanReceiver::IsRunning() const { return is_running_; }

ErrorCode CanReceiver::Start() {
  if (is_init_ == false) {
    return ErrorCode::CANBUS_ERROR;
  }
  is_running_ = true;

  thread_.reset(new std::thread([this] { RecvThreadFunc(); }));
  if (thread_ == nullptr) {
    AERROR << "Unable to create can client receiver thread.";
    return ErrorCode::CANBUS_ERROR;
  }
  return ErrorCode::OK;
}

void CanReceiver::Stop() {
  if (IsRunning()) {
    AINFO << "Stopping can client receiver ...";
    is_running_ = false;
    if (thread_ != nullptr && thread_->joinable()) {
      thread_->join();
    }
    thread_.reset();
  } else {
    AINFO << "Can client receiver is not running.";
  }
  AINFO << "Can client receiver stopped [ok].";
}

}  // namespace canbus
}  // namespace apollo
