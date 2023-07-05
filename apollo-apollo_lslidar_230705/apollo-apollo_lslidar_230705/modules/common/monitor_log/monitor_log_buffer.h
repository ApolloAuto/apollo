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
 * @file monitor_log_buffer.h
 * @brief The class of MonitorLogBuffer
 */

#pragma once

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"
#include "modules/common/monitor_log/monitor_logger.h"
#include "modules/common_msgs/monitor_msgs/monitor_log.pb.h"

/**
 * @namespace apollo::common::monitor
 * @brief apollo::common::monitor
 */
namespace apollo {
namespace common {
namespace monitor {

#define REG_MSG_TYPE(TYPE)                            \
  MonitorLogBuffer &TYPE(const std::string &msg) {    \
    AddMonitorMsgItem(MonitorMessageItem::TYPE, msg); \
    Publish();                                        \
    return *this;                                     \
  }                                                   \
  MonitorLogBuffer &TYPE() {                          \
    level_ = MonitorMessageItem::TYPE;                \
    return *this;                                     \
  }

/**
 * class MonitorLogBuffer
 *
 * @brief This class help collect MonitorMessage pb to monitor topic.
 * The messages can be published automatically when the MonitorBuffer object's
 * destructor is called, or can be published by calling function Publish().
 */
class MonitorLogBuffer {
 public:
  /**
   * @brief The constructor of MonitorBuffer.
   * @param a Monitor instance pointer;
   */
  explicit MonitorLogBuffer(const MonitorMessageItem::MessageSource &source);

  virtual ~MonitorLogBuffer();

  /**
   * @brief record an INFO type message
   */
  REG_MSG_TYPE(INFO);

  /**
   * @brief record a WARN type message
   */
  REG_MSG_TYPE(WARN);

  /**
   * @brief record an ERROR type message
   */
  REG_MSG_TYPE(ERROR);

  /**
   * @brief record a FATAL type message
   */
  REG_MSG_TYPE(FATAL);

  /**
   * @brief Add monitor message with MonitorMessageItem::LogLevel
   * @param log_level defined in modules/common/monitor/proto/monitor.proto
   * @param msg the string to send to monitor
   */
  void AddMonitorMsgItem(const MonitorMessageItem::LogLevel log_level,
                         const std::string &msg);

  /**
   * @brief publish the monitor messages
   */
  void Publish();

 private:
  MonitorLogger *logger_ = MonitorLogger::Instance();
  MonitorMessageItem::LogLevel level_ = MonitorMessageItem::INFO;
  std::vector<MessageItem> monitor_msg_items_;
  MonitorMessageItem::MessageSource source_;

  FRIEND_TEST(MonitorBufferTest, RegisterMacro);
  FRIEND_TEST(MonitorBufferTest, AddMonitorMsgItem);
  FRIEND_TEST(MonitorBufferTest, Operator);
};

}  // namespace monitor
}  // namespace common
}  // namespace apollo
