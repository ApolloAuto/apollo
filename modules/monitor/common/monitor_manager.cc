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

#include "modules/monitor/common/monitor_manager.h"

#include "gflags/gflags.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

DEFINE_string(monitor_conf_path, "modules/monitor/conf/monitor_conf.pb.txt",
              "Path of the monitor config file.");

namespace apollo {
namespace monitor {

using apollo::canbus::Chassis;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::LookupOrInsert;

MonitorManager::MonitorManager() :
  logger_(apollo::common::monitor::MonitorMessageItem::MONITOR),
  log_buffer_(&logger_) {
  CHECK(apollo::common::util::GetProtoFromASCIIFile(FLAGS_monitor_conf_path,
                                                    &config_));
}

apollo::common::monitor::MonitorLogBuffer &MonitorManager::LogBuffer() {
  return instance()->log_buffer_;
}

const MonitorConf &MonitorManager::GetConfig() {
  return instance()->config_;
}

void MonitorManager::InitFrame(const double current_time) {
  // Clear old summaries.
  for (auto &module : *GetStatus()->mutable_modules()) {
    module.second.set_summary(Summary::UNKNOWN);
    module.second.clear_msg();
  }
  for (auto &hardware : *GetStatus()->mutable_hardware()) {
    hardware.second.set_summary(Summary::UNKNOWN);
    hardware.second.clear_msg();
  }

  // Get current DrivingMode, which will affect how we monitor modules.
  instance()->in_autonomous_driving_ = false;
  auto* adapter = CHECK_NOTNULL(AdapterManager::GetChassis());
  adapter->Observe();
  if (!adapter->Empty()) {
    const auto& chassis = adapter->GetLatestObserved();
    // Ignore old messages which is likely from replaying.
    instance()->in_autonomous_driving_ =
        chassis.driving_mode() == Chassis::COMPLETE_AUTO_DRIVE &&
        chassis.header().timestamp_sec() + FLAGS_system_status_lifetime_seconds
            >= current_time;
  }
}

SystemStatus *MonitorManager::GetStatus() {
  return &instance()->status_;
}

HardwareStatus *MonitorManager::GetHardwareStatus(
    const std::string &hardware_name) {
  return &LookupOrInsert(GetStatus()->mutable_hardware(), hardware_name, {});
}

ModuleStatus *MonitorManager::GetModuleStatus(const std::string &module_name) {
  return &LookupOrInsert(GetStatus()->mutable_modules(), module_name, {});
}

bool MonitorManager::IsInAutonomousDriving() {
  return instance()->in_autonomous_driving_;
}

}  // namespace monitor
}  // namespace apollo
