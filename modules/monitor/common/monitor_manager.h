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

#ifndef MODULES_MONITOR_COMMON_MONITOR_MANAGER_H_
#define MODULES_MONITOR_COMMON_MONITOR_MANAGER_H_

#include <string>

#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/monitor/proto/monitor_conf.pb.h"
#include "modules/monitor/proto/system_status.pb.h"

/**
 * @namespace apollo::monitor
 * @brief apollo::monitor
 */
namespace apollo {
namespace monitor {

class MonitorManager {
 public:
  static const MonitorConf &GetConfig();
  static SystemStatus *GetStatus();
  static HardwareStatus *GetHardwareStatus(const std::string &hardware_name);
  static ModuleStatus *GetModuleStatus(const std::string &module_name);
  static apollo::common::monitor::MonitorLogBuffer &LogBuffer();

 private:
  MonitorConf config_;
  SystemStatus status_;
  apollo::common::monitor::MonitorLogger logger_;
  apollo::common::monitor::MonitorLogBuffer log_buffer_;

  DECLARE_SINGLETON(MonitorManager);
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_COMMON_MONITOR_MANAGER_H_
