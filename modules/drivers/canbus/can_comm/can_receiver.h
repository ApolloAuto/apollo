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

/**
 * @file
 * @brief Defines CanReceiver class.
 */

#pragma once

#include <atomic>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "modules/common_msgs/basic_msgs/error_code.pb.h"

#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class CanReceiver
 * @brief CAN receiver.
 */
template <typename SensorType>
class CanReceiver {
 public:
  /**
   * @brief Constructor.
   */
  CanReceiver() = default;

  /**
   * @brief Destructor.
   */
  virtual ~CanReceiver() = default;

  /**
   * @brief Initialize by a CAN client, message manager.
   * @param can_client The CAN client to use for receiving messages.
   * @param pt_manager The message manager which can parse and
   *        get protocol data by message id.
   * @param enable_log If log the essential information during running.
   * @return An error code indicating the status of this initialization.
   */
  common::ErrorCode Init(CanClient *can_client,
                         MessageManager<SensorType> *pt_manager,
                         bool enable_log);

  /**
   * @brief Get the working status of this CAN receiver.
   *        To check if it is running.
   * @return If this CAN receiver is running.
   */
  bool IsRunning() const;

  /**
   * @brief Get the receive status in once receive buf.
   *        To check if it is running.
   * @return If this CAN receiver is running.
   */
  bool IsFinishRecvOnce() const;

  /**
   * @brief Start the CAN receiver.
   * @return The error code indicating the status of this action.
   */
  ::apollo::common::ErrorCode Start();

  /**
   * @brief Stop the CAN receiver.
   */
  void Stop();

 private:
  void RecvThreadFunc();

  int32_t Start(bool is_blocked);

 private:
  std::atomic<bool> is_running_ = {false};
  std::atomic<bool> is_finish_recv_once_ = {false};
  // CanClient, MessageManager pointer life is managed by outer program
  CanClient *can_client_ = nullptr;
  MessageManager<SensorType> *pt_manager_ = nullptr;
  bool enable_log_ = false;
  bool is_init_ = false;
  std::future<void> async_result_;

  DISALLOW_COPY_AND_ASSIGN(CanReceiver);
};

template <typename SensorType>
::apollo::common::ErrorCode CanReceiver<SensorType>::Init(
    CanClient *can_client, MessageManager<SensorType> *pt_manager,
    bool enable_log) {
  can_client_ = can_client;
  pt_manager_ = pt_manager;
  enable_log_ = enable_log;
  if (can_client_ == nullptr) {
    AERROR << "Invalid can client.";
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }
  if (pt_manager_ == nullptr) {
    AERROR << "Invalid protocol manager.";
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }
  is_init_ = true;
  return ::apollo::common::ErrorCode::OK;
}

template <typename SensorType>
void CanReceiver<SensorType>::RecvThreadFunc() {
  AINFO << "Can client receiver thread starts.";
  CHECK_NOTNULL(can_client_);
  CHECK_NOTNULL(pt_manager_);

  int32_t receive_error_count = 0;
  int32_t receive_none_count = 0;
  const int32_t ERROR_COUNT_MAX = 10;
  auto default_period = 10 * 1000;

  while (IsRunning()) {
    is_finish_recv_once_.exchange(false);
    ADEBUG << "is_finish_recv_once_ 1 is " << is_finish_recv_once_.load();
    std::vector<CanFrame> buf;
    int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
    if (can_client_->Receive(&buf, &frame_num) !=
        ::apollo::common::ErrorCode::OK) {
      LOG_IF_EVERY_N(ERROR, receive_error_count++ > ERROR_COUNT_MAX,
                     ERROR_COUNT_MAX)
          << "Received " << receive_error_count << " error messages.";
      cyber::USleep(default_period);
      continue;
    }
    receive_error_count = 0;

    if (buf.size() != static_cast<size_t>(frame_num)) {
      AERROR_EVERY(100) << "Receiver buf size [" << buf.size()
                        << "] does not match can_client returned length["
                        << frame_num << "].";
    }

    if (frame_num == 0) {
      LOG_IF_EVERY_N(ERROR, receive_none_count++ > ERROR_COUNT_MAX,
                     ERROR_COUNT_MAX)
          << "Received " << receive_none_count << " empty messages.";
      cyber::USleep(default_period);
      continue;
    }
    receive_none_count = 0;

    for (const auto &frame : buf) {
      uint8_t len = frame.len;
      uint32_t uid = frame.id;
      const uint8_t *data = frame.data;
      pt_manager_->Parse(uid, data, len);
      if (enable_log_) {
        AINFO << "recv_can_frame#" << frame.CanFrameString();
      }
    }
    is_finish_recv_once_.exchange(true);
    ADEBUG << "is_finish_recv_once_ 2 is " << is_finish_recv_once_.load();
    cyber::Yield();
  }
  AINFO << "Can client receiver thread stopped.";
}

template <typename SensorType>
bool CanReceiver<SensorType>::IsRunning() const {
  return is_running_.load();
}

template <typename SensorType>
bool CanReceiver<SensorType>::IsFinishRecvOnce() const {
  ADEBUG << "is_finish_recv_once_ state is " << is_finish_recv_once_.load();
  return is_finish_recv_once_.load();
}

template <typename SensorType>
::apollo::common::ErrorCode CanReceiver<SensorType>::Start() {
  if (is_init_ == false) {
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }
  is_running_.exchange(true);

  async_result_ = cyber::Async(&CanReceiver<SensorType>::RecvThreadFunc, this);
  return ::apollo::common::ErrorCode::OK;
}

template <typename SensorType>
void CanReceiver<SensorType>::Stop() {
  if (IsRunning()) {
    AINFO << "Stopping can client receiver ...";
    is_running_.exchange(false);
    async_result_.wait();
  } else {
    AINFO << "Can client receiver is not running.";
  }
  AINFO << "Can client receiver stopped [ok].";
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
