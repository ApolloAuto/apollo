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
#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"

DEFINE_string(monitor_conf_path, "modules/monitor/conf/monitor_conf.pb.txt",
              "Path of the monitor config file.");

namespace apollo {
namespace monitor {

using apollo::common::util::LookupOrInsert;

MonitorManager::MonitorManager() {
  CHECK(apollo::common::util::GetProtoFromASCIIFile(FLAGS_monitor_conf_path,
                                                    &config_));
}

const MonitorConf &MonitorManager::GetConfig() {
  return instance()->config_;
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

}  // namespace monitor
}  // namespace apollo
