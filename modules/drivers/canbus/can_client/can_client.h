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
 * @brief Defines the CanFrame struct and CanClient interface.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class CanFrame
 * @brief The class which defines the information to send and receive.
 */
struct CanFrame {
  /// Message id
  uint32_t id;
  /// Message length
  uint8_t len;
  /// Message content
  uint8_t data[8];
  /// Time stamp
  struct timeval timestamp;

  /**
   * @brief Constructor
   */
  CanFrame() : id(0), len(0), timestamp{0} {
    std::memset(data, 0, sizeof(data));
  }

  /**
   * @brief CanFrame string including essential information about the message.
   * @return The info string.
   */
  std::string CanFrameString() const {
    std::stringstream output_stream("");
    output_stream << "id:0x" << Byte::byte_to_hex(id)
                  << ",len:" << static_cast<int>(len) << ",data:";
    for (uint8_t i = 0; i < len; ++i) {
      output_stream << Byte::byte_to_hex(data[i]);
    }
    output_stream << ",";
    return output_stream.str();
  }
};

const int CAN_RESULT_SUCC = 0;
const int CAN_ERROR_BASE = 2000;
const int CAN_ERROR_OPEN_DEVICE_FAILED = CAN_ERROR_BASE + 1;
const int CAN_ERROR_FRAME_NUM = CAN_ERROR_BASE + 2;
const int CAN_ERROR_SEND_FAILED = CAN_ERROR_BASE + 3;
const int CAN_ERROR_RECV_FAILED = CAN_ERROR_BASE + 4;

/**
 * @class CanClient
 * @brief The class which defines the CAN client to send and receive message.
 */
class CanClient {
 public:
  /**
   * @brief Constructor
   */
  CanClient() = default;

  /**
   * @brief Destructor
   */
  virtual ~CanClient() = default;

  /**
   * @brief Initialize the CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  virtual bool Init(const CANCardParameter &parameter) = 0;

  /**
   * @brief Start the CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  virtual apollo::common::ErrorCode Start() = 0;

  /**
   * @brief Stop the CAN client.
   */
  virtual void Stop() = 0;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  virtual apollo::common::ErrorCode Send(const std::vector<CanFrame> &frames,
                                         int32_t *const frame_num) = 0;

  /**
   * @brief Send a single message.
   * @param frames A single-element vector containing only one message.
   * @return The status of the sending single message action which is defined by
   *         apollo::common::ErrorCode.
   */
  virtual apollo::common::ErrorCode SendSingleFrame(
      const std::vector<CanFrame> &frames) {
    CHECK_EQ(frames.size(), 1)
        << "frames size not equal to 1, actual frame size :" << frames.size();
    int32_t n = 1;
    return Send(frames, &n);
  }

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  virtual apollo::common::ErrorCode Receive(std::vector<CanFrame> *const frames,
                                            int32_t *const frame_num) = 0;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  virtual std::string GetErrorString(const int32_t status) = 0;

 protected:
  /// The CAN client is started.
  bool is_started_ = false;
};

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
