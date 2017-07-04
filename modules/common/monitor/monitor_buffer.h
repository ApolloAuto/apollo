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
 * @file monitor_buffer.h
 * @brief The class of MonitorBuffer
 */

#ifndef MODULES_MONITOR_MONITOR_BUFFER_H_
#define MODULES_MONITOR_MONITOR_BUFFER_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "modules/common/monitor/proto/monitor.pb.h"

/**
 * @namespace apollo::common::monitor
 * @brief apollo::common::monitor
 */
namespace apollo {
namespace common {
namespace monitor {

using MessageItem = std::pair<MonitorMessageItem::LogLevel, std::string>;

class Monitor;

#define REG_MSG_TYPE(TYPE)                            \
  MonitorBuffer& TYPE(const std::string& msg) {       \
    AddMonitorMsgItem(MonitorMessageItem::TYPE, msg); \
    return *this;                                     \
  }                                                   \
  MonitorBuffer& TYPE() {                             \
    level_ = MonitorMessageItem::TYPE;                \
    return *this;                                     \
  }

/**
 * class MonitorBuffer
 *
 * @brief This class help collect MonitorMessage pb to monitor topic.
 * The messages can be published automatically when the MonitorBuffer object's
 * destructor is called, or can be published by calling function Publish().
 */
class MonitorBuffer {
 public:
  /**
   * @brief The constructor of MonitorBuffer.
   * @param a Monitor instance pointer;
   */
  explicit MonitorBuffer(Monitor* monitor);

  virtual ~MonitorBuffer();

  /**
   * @brief print a log trace for the last recorded message.
   */
  void PrintLog();

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
                         const std::string& msg);

  /**
   * @brief overload operator << to help join messages
   */
  template <typename T>
  MonitorBuffer& operator<<(const T& msg) {
    if (monitor_msg_items_.empty() ||
        monitor_msg_items_.back().first != level_) {
      AddMonitorMsgItem(level_, std::to_string(msg));
    } else {
      monitor_msg_items_.back().second += std::to_string(msg);
    }
    return *this;
  }

  /**
   * @brief overload operator << to help join string messages
   */
  MonitorBuffer& operator<<(const std::string& msg);

  /**
   * @brief overload operator << to help join char messages
   */
  MonitorBuffer& operator<<(const char* msg);

  /**
   * @brief publish the monitor messages
   */
  void Publish();

 private:
  Monitor* monitor_ = nullptr;
  MonitorMessageItem::LogLevel level_ = MonitorMessageItem::INFO;
  std::vector<MessageItem> monitor_msg_items_;

  FRIEND_TEST(MonitorBufferTest, RegisterMacro);
  FRIEND_TEST(MonitorBufferTest, AddMonitorMsgItem);
  FRIEND_TEST(MonitorBufferTest, Operator);
};

}  // namespace monitor
}  // namespace common
}  // namespace apollo

#endif  // MODULES_MONITOR_MONITOR_BUFFER_H_
