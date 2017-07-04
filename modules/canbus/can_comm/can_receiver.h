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

#ifndef MODULES_CANBUS_CAN_COMM_CAN_RECEIVER_H_
#define MODULES_CANBUS_CAN_COMM_CAN_RECEIVER_H_

#include <memory>
#include <thread>

#include "modules/canbus/can_client/can_client.h"
#include "modules/canbus/vehicle/message_manager.h"
#include "modules/common/macro.h"
#include "modules/common/proto/error_code.pb.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class CanReceiver
 * @brief CAN receiver.
 */
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
  ::apollo::common::ErrorCode Init(CanClient* can_client,
                                   MessageManager* pt_manager, bool enable_log);

  /**
   * @brief Get the working status of this CAN receiver.
   *        To check if it is running.
   * @return If this CAN receiver is running.
   */
  bool IsRunning() const;

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
  std::unique_ptr<std::thread> thread_;
  bool is_running_ = false;
  // CanClient, MessageManager pointer life is managed by outer program
  CanClient* can_client_ = nullptr;
  MessageManager* pt_manager_ = nullptr;
  bool enable_log_ = false;
  bool is_init_ = false;

  DISALLOW_COPY_AND_ASSIGN(CanReceiver);
};

}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_CAN_COMM_CAN_RECEIVER_H_
