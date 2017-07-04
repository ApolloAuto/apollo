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

#ifndef MODULES_CANBUS_CAN_COMM_CAN_SENDER_H_
#define MODULES_CANBUS_CAN_COMM_CAN_SENDER_H_

#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/canbus/can_client/can_client.h"
#include "modules/canbus/vehicle/protocol_data.h"
#include "modules/common/macro.h"
#include "modules/common/proto/error_code.pb.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class SenderMessage
 * @brief This class defines the message to send.
 */
class SenderMessage {
 public:
  /**
   * @brief Constructor which takes message ID and protocol data.
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   */
  SenderMessage(const uint32_t message_id, ProtocolData* protocol_data);

  /**
   * @brief Constructor which takes message ID and protocol data and
   *        and indicator whether to initialize all bits of the .
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   * @param init_with_one If it is true, then initialize all bits in
   *        the protocal data as one.
   */
  SenderMessage(const uint32_t message_id, ProtocolData* protocol_data,
                bool init_with_one);

  /**
   * @brief Destructor.
   */
  ~SenderMessage() = default;

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
  ProtocolData* protocol_data_ = nullptr;

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
  ::apollo::common::ErrorCode Init(CanClient* can_client, bool enable_log);

  /**
   * @brief Add a message with its ID, protocol data.
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   * @param init_with_one If it is true, then initialize all bits in
   *        the protocal data as one. By default, it is false.
   */
  void AddMessage(uint32_t message_id, ProtocolData* protocol_data,
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

  bool NeedSend(const SenderMessage& msg, const int32_t delta_period);
  bool is_init_ = false;
  bool is_running_ = false;

  CanClient* can_client_ = nullptr;  // Owned by global canbus.cc
  std::vector<SenderMessage> send_messages_;
  std::unique_ptr<std::thread> thread_;
  bool enable_log_ = false;

  DISALLOW_COPY_AND_ASSIGN(CanSender);
};

}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_CAN_COMM_CAN_SENDER_H_
