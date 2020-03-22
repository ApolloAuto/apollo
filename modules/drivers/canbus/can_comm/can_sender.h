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
 * @brief Defines SenderMessage class and CanSender class.
 */

#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "cyber/common/macros.h"

#include "cyber/common/log.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class SenderMessage
 * @brief This class defines the message to send.
 */
template <typename SensorType>
class SenderMessage {
 public:
  /**
   * @brief Constructor which takes message ID and protocol data.
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   */
  SenderMessage(const uint32_t message_id,
                ProtocolData<SensorType> *protocol_data);

  /**
   * @brief Constructor which takes message ID and protocol data and
   *        and indicator whether to initialize all bits of the .
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   * @param init_with_one If it is true, then initialize all bits in
   *        the protocol data as one.
   */
  SenderMessage(const uint32_t message_id,
                ProtocolData<SensorType> *protocol_data, bool init_with_one);

  /**
   * @brief Destructor.
   */
  virtual ~SenderMessage() = default;

  /**
   * @brief Update the current period for sending messages by a difference.
   * @param delta_period Update the current period by reducing delta_period.
   */
  void UpdateCurrPeriod(const int32_t delta_period);

  /**
   * @brief Update the protocol data. But the updating process depends on
   *        the real type of protocol data which inherites ProtocolData.
   */
  void Update();

  /**
   * @brief Get the CAN frame to send.
   * @return The CAN frame to send.
   */
  struct CanFrame CanFrame();

  /**
   * @brief Get the message ID.
   * @return The message ID.
   */
  uint32_t message_id() const;

  /**
   * @brief Get the current period to send messages. It may be different from
   *        the period from protocol data by updating.
   * @return The current period.
   */
  int32_t curr_period() const;

 private:
  uint32_t message_id_ = 0;
  ProtocolData<SensorType> *protocol_data_ = nullptr;

  int32_t period_ = 0;
  int32_t curr_period_ = 0;

 private:
  static std::mutex mutex_;
  struct CanFrame can_frame_to_send_;
  struct CanFrame can_frame_to_update_;
};

/**
 * @class CanSender
 * @brief CAN sender.
 */
template <typename SensorType>
class CanSender {
 public:
  /**
   * @brief Constructor.
   */
  CanSender() = default;

  /**
   * @brief Destructor.
   */
  virtual ~CanSender() = default;

  /**
   * @brief Initialize by a CAN client based on its brand.
   * @param can_client The CAN client to use for sending messages.
   * @param enable_log whether enable record the send can frame log
   * @return An error code indicating the status of this initialization.
   */
  common::ErrorCode Init(CanClient *can_client, bool enable_log);

  /**
   * @brief Add a message with its ID, protocol data.
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   * @param init_with_one If it is true, then initialize all bits in
   *        the protocol data as one. By default, it is false.
   */
  void AddMessage(uint32_t message_id, ProtocolData<SensorType> *protocol_data,
                  bool init_one = false);

  /**
   * @brief Start the CAN sender.
   * @return The error code indicating the status of this action.
   */
  apollo::common::ErrorCode Start();

  /*
   * @brief Update the protocol data based the types.
   */
  void Update();

  /**
   * @brief Stop the CAN sender.
   */
  void Stop();

  /**
   * @brief Get the working status of this CAN sender.
   *        To check if it is running.
   * @return If this CAN sender is running.
   */
  bool IsRunning() const;
  bool enable_log() const;

  FRIEND_TEST(CanSenderTest, OneRunCase);

 private:
  void PowerSendThreadFunc();

  bool NeedSend(const SenderMessage<SensorType> &msg,
                const int32_t delta_period);
  bool is_init_ = false;
  bool is_running_ = false;

  CanClient *can_client_ = nullptr;  // Owned by global canbus.cc
  std::vector<SenderMessage<SensorType>> send_messages_;
  std::unique_ptr<std::thread> thread_;
  bool enable_log_ = false;

  DISALLOW_COPY_AND_ASSIGN(CanSender);
};

const uint32_t kSenderInterval = 6000;

template <typename SensorType>
std::mutex SenderMessage<SensorType>::mutex_;

template <typename SensorType>
SenderMessage<SensorType>::SenderMessage(
    const uint32_t message_id, ProtocolData<SensorType> *protocol_data)
    : SenderMessage(message_id, protocol_data, false) {}

template <typename SensorType>
SenderMessage<SensorType>::SenderMessage(
    const uint32_t message_id, ProtocolData<SensorType> *protocol_data,
    bool init_with_one)
    : message_id_(message_id), protocol_data_(protocol_data) {
  if (init_with_one) {
    for (int32_t i = 0; i < protocol_data->GetLength(); ++i) {
      can_frame_to_update_.data[i] = 0xFF;
    }
  }
  int32_t len = protocol_data_->GetLength();

  can_frame_to_update_.id = message_id_;
  can_frame_to_update_.len = static_cast<uint8_t>(len);

  period_ = protocol_data_->GetPeriod();
  curr_period_ = period_;

  Update();
}

template <typename SensorType>
void SenderMessage<SensorType>::UpdateCurrPeriod(const int32_t period_delta) {
  curr_period_ -= period_delta;
  if (curr_period_ <= 0) {
    curr_period_ = period_;
  }
}

template <typename SensorType>
void SenderMessage<SensorType>::Update() {
  if (protocol_data_ == nullptr) {
    AERROR << "Attention: ProtocolData is nullptr!";
    return;
  }
  protocol_data_->UpdateData(can_frame_to_update_.data);

  std::lock_guard<std::mutex> lock(mutex_);
  can_frame_to_send_ = can_frame_to_update_;
}

template <typename SensorType>
uint32_t SenderMessage<SensorType>::message_id() const {
  return message_id_;
}

template <typename SensorType>
struct CanFrame SenderMessage<SensorType>::CanFrame() {
  std::lock_guard<std::mutex> lock(mutex_);
  return can_frame_to_send_;
}

template <typename SensorType>
int32_t SenderMessage<SensorType>::curr_period() const {
  return curr_period_;
}

template <typename SensorType>
void CanSender<SensorType>::PowerSendThreadFunc() {
  CHECK_NOTNULL(can_client_);
  sched_param sch;
  sch.sched_priority = 99;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

  const int32_t INIT_PERIOD = 5000;  // 5ms
  int32_t delta_period = INIT_PERIOD;
  int32_t new_delta_period = INIT_PERIOD;

  int64_t tm_start = 0;
  int64_t tm_end = 0;
  int64_t sleep_interval = 0;

  AINFO << "Can client sender thread starts.";

  while (is_running_) {
    tm_start = absl::ToUnixMicros(common::time::Clock::Now());
    new_delta_period = INIT_PERIOD;

    for (auto &message : send_messages_) {
      bool need_send = NeedSend(message, delta_period);
      message.UpdateCurrPeriod(delta_period);
      new_delta_period = std::min(new_delta_period, message.curr_period());

      if (!need_send) {
        continue;
      }
      std::vector<CanFrame> can_frames;
      CanFrame can_frame = message.CanFrame();
      can_frames.push_back(can_frame);
      if (can_client_->SendSingleFrame(can_frames) != common::ErrorCode::OK) {
        AERROR << "Send msg failed:" << can_frame.CanFrameString();
      }
      if (enable_log()) {
        ADEBUG << "send_can_frame#" << can_frame.CanFrameString();
      }
    }
    delta_period = new_delta_period;
    tm_end = absl::ToUnixMicros(common::time::Clock::Now());
    sleep_interval = delta_period - (tm_end - tm_start);

    if (sleep_interval > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_interval));
    } else {
      // do not sleep
      AWARN << "Too much time for calculation: " << tm_end - tm_start
            << "us is more than minimum period: " << delta_period << "us";
    }
  }
  AINFO << "Can client sender thread stopped!";
}

template <typename SensorType>
common::ErrorCode CanSender<SensorType>::Init(CanClient *can_client,
                                              bool enable_log) {
  if (is_init_) {
    AERROR << "Duplicated Init request.";
    return common::ErrorCode::CANBUS_ERROR;
  }
  if (can_client == nullptr) {
    AERROR << "Invalid can client.";
    return common::ErrorCode::CANBUS_ERROR;
  }
  is_init_ = true;
  can_client_ = can_client;
  enable_log_ = enable_log;
  return common::ErrorCode::OK;
}

template <typename SensorType>
void CanSender<SensorType>::AddMessage(uint32_t message_id,
                                       ProtocolData<SensorType> *protocol_data,
                                       bool init_with_one) {
  if (protocol_data == nullptr) {
    AERROR << "invalid protocol data.";
    return;
  }
  send_messages_.emplace_back(
      SenderMessage<SensorType>(message_id, protocol_data, init_with_one));
  AINFO << "Add send message:" << std::hex << message_id;
}

template <typename SensorType>
common::ErrorCode CanSender<SensorType>::Start() {
  if (is_running_) {
    AERROR << "Cansender has already started.";
    return common::ErrorCode::CANBUS_ERROR;
  }
  is_running_ = true;
  thread_.reset(new std::thread([this] { PowerSendThreadFunc(); }));

  return common::ErrorCode::OK;
}

template <typename SensorType>
void CanSender<SensorType>::Update() {
  for (auto &message : send_messages_) {
    message.Update();
  }
}

template <typename SensorType>
void CanSender<SensorType>::Stop() {
  if (is_running_) {
    AINFO << "Stopping can sender ...";
    is_running_ = false;
    if (thread_ != nullptr && thread_->joinable()) {
      thread_->join();
    }
    thread_.reset();
  } else {
    AERROR << "CanSender is not running.";
  }

  AINFO << "Can client sender stopped [ok].";
}

template <typename SensorType>
bool CanSender<SensorType>::IsRunning() const {
  return is_running_;
}

template <typename SensorType>
bool CanSender<SensorType>::enable_log() const {
  return enable_log_;
}

template <typename SensorType>
bool CanSender<SensorType>::NeedSend(const SenderMessage<SensorType> &msg,
                                     const int32_t delta_period) {
  return msg.curr_period() <= delta_period;
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
