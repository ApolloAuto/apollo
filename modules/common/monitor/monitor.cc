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

#include "modules/common/monitor/monitor.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace common {
namespace monitor {

using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;

void Monitor::Publish(const std::vector<MessageItem> &messages) const {
  // compose a monitor message
  if (messages.empty()) {
    return;
  }
  MonitorMessage monitor_msg;

  for (const auto &msg_item : messages) {
    MonitorMessageItem *monitor_msg_item = monitor_msg.add_item();
    monitor_msg_item->set_source(source_);
    monitor_msg_item->set_log_level(msg_item.first);
    monitor_msg_item->set_msg(msg_item.second);
  }

  // publish monitor messages
  DoPublish(&monitor_msg);
}

void Monitor::DoPublish(MonitorMessage *message) const {
  DCHECK(AdapterManager::Initialized())
      << "AdapterManager must be initialized before using monitor.";
  AdapterManager::FillMonitorHeader("monitor", message);
  AdapterManager::PublishMonitor(*message);
}

}  // namespace monitor
}  // namespace common
}  // namespace apollo
