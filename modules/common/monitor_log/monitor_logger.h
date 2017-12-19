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
 * @file monitor_logger.h
 * @brief The class of MonitorLogger
 */

#ifndef MODULES_COMMON_MONITOR_LOG_MONITOR_LOGGER_H_
#define MODULES_COMMON_MONITOR_LOG_MONITOR_LOGGER_H_

#include <string>
#include <utility>
#include <vector>

#include "modules/common/monitor_log/proto/monitor_log.pb.h"

/**
 * @namespace apollo::common::monitor
 * @brief apollo::common::monitor
 */
namespace apollo {
namespace common {
namespace monitor {

using MessageItem = std::pair<MonitorMessageItem::LogLevel, std::string>;

/**
 * class MonitorLogger
 *
 * @brief This class help collect and publish MonitorMessage pb to monitor
 * topic. A module who wants to publish message can use macro
 * `MONITOR(log_level, log_msg)` to record messages, and call
 * Publish to broadcast the message to other modules.
 */
class MonitorLogger {
 public:
  /**
   * @brief Construct the monitor with the source of the monitor messages. The
   * source is usually the module name who publish the monitor messages.
   * @param source the source of the monitor messages.
   */
  explicit MonitorLogger(const MonitorMessageItem::MessageSource &source)
      : source_(source) {}
  virtual ~MonitorLogger() = default;

  /**
   * @brief Publish the messages.
   * @param messages a list of messages for
   */
  virtual void Publish(const std::vector<MessageItem> &messages) const;

 private:
  virtual void DoPublish(MonitorMessage *message) const;

  MonitorMessageItem::MessageSource source_;
};

}  // namespace monitor
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_MONITOR_LOG_MONITOR_LOGGER_H_
