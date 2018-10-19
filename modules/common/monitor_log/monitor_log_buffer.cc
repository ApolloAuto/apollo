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

#include "modules/common/monitor_log/monitor_log_buffer.h"

#include "cyber/common/log.h"
#include "modules/common/monitor_log/monitor_logger.h"

namespace apollo {
namespace common {
namespace monitor {

MonitorLogBuffer::MonitorLogBuffer(
    const MonitorMessageItem::MessageSource &source)
    : source_(source) {}

void MonitorLogBuffer::Publish() {
  if (!monitor_msg_items_.empty()) {
    logger_->Publish(source_, monitor_msg_items_);
    monitor_msg_items_.clear();
    level_ = MonitorMessageItem::INFO;
  }
}

MonitorLogBuffer::~MonitorLogBuffer() { Publish(); }

void MonitorLogBuffer::AddMonitorMsgItem(
    const MonitorMessageItem::LogLevel log_level, const std::string &msg) {
  level_ = log_level;
  monitor_msg_items_.push_back(std::make_pair(log_level, msg));
}

}  // namespace monitor
}  // namespace common
}  // namespace apollo
