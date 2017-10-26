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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/monitor/monitor.h"
#include "modules/common/monitor/monitor_buffer.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace common {
namespace monitor {

using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;

MonitorBuffer::MonitorBuffer(Monitor *monitor) : monitor_(monitor) {}

void MonitorBuffer::PrintLog() {
  if (monitor_msg_items_.empty()) {
    return;
  }
  const auto level = monitor_msg_items_.back().first;
  const auto &msg = monitor_msg_items_.back().second;
  switch (level) {
    case MonitorMessageItem::INFO:
      AINFO << msg;
      break;
    case MonitorMessageItem::WARN:
      AWARN << msg;
      break;
    case MonitorMessageItem::ERROR:
      AERROR << msg;
      break;
    case MonitorMessageItem::FATAL:
      AFATAL << msg;
      break;
    default:
      AERROR << "[unknown monitor level]: " << msg;
  }
}

void MonitorBuffer::Publish() {
  if (!monitor_msg_items_.empty() && monitor_) {
    monitor_->Publish(monitor_msg_items_);
    monitor_msg_items_.clear();
    level_ = MonitorMessageItem::INFO;
  }
}

MonitorBuffer &MonitorBuffer::operator<<(const std::string &msg) {
  if (monitor_msg_items_.empty() || monitor_msg_items_.back().first != level_) {
    AddMonitorMsgItem(level_, msg);
  } else {
    monitor_msg_items_.back().second += msg;
  }
  return *this;
}

MonitorBuffer &MonitorBuffer::operator<<(const char *msg) {
  if (msg) {
    std::string msg_str(msg);
    return operator<<(msg_str);
  } else {
    return *this;
  }
}

MonitorBuffer::~MonitorBuffer() { Publish(); }

void MonitorBuffer::AddMonitorMsgItem(
    const MonitorMessageItem::LogLevel log_level, const std::string &msg) {
  level_ = log_level;
  monitor_msg_items_.push_back(std::make_pair(log_level, msg));
}

}  // namespace monitor
}  // namespace common
}  // namespace apollo
