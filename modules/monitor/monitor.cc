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
#include "modules/monitor/monitor.h"

#include "modules/common/time/time.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/hardware/esdcan_monitor.h"
#include "modules/monitor/hardware/gps_monitor.h"
#include "modules/monitor/hardware/resource_monitor.h"
#include "modules/monitor/hardware/socket_can_monitor.h"
#include "modules/monitor/software/channel_monitor.h"
#include "modules/monitor/software/functional_safety_monitor.h"
#include "modules/monitor/software/localization_monitor.h"
#include "modules/monitor/software/process_monitor.h"
#include "modules/monitor/software/recorder_monitor.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_bool(enable_functional_safety, true,
            "Whether to enable functional safety check.");

namespace apollo {
namespace monitor {

bool Monitor::Init() {
  MonitorManager::Instance()->Init(node_);

  // Only the one CAN card corresponding to current mode will take effect.
  runners_.emplace_back(new EsdCanMonitor());
  runners_.emplace_back(new SocketCanMonitor());
  // To enable the GpsMonitor, you must add FLAGS_gps_component_name to the
  // mode's monitored_components.
  runners_.emplace_back(new GpsMonitor());
  // To enable the LocalizationMonitor, you must add
  // FLAGS_localization_component_name to the mode's monitored_components.
  runners_.emplace_back(new LocalizationMonitor());
  // Monitor if processes are running.
  runners_.emplace_back(new ProcessMonitor());
  // Monitor if channel messages are updated in time.
  runners_.emplace_back(new ChannelMonitor());
  // Monitor if resources are sufficient.
  runners_.emplace_back(new ResourceMonitor());

  // Monitor all changes made by each sub-monitor, and summarize to a final
  // overall status.
  runners_.emplace_back(new SummaryMonitor());
  // Check functional safety according to the summary.
  if (FLAGS_enable_functional_safety) {
    runners_.emplace_back(new FunctionalSafetyMonitor());
  }

  return true;
}

bool Monitor::Proc() {
  const double current_time = apollo::common::time::Clock::NowInSeconds();
  if (!MonitorManager::Instance()->StartFrame(current_time)) {
    return false;
  }
  for (auto& runner : runners_) {
    runner->Tick(current_time);
  }
  MonitorManager::Instance()->EndFrame();

  return true;
}

}  // namespace monitor
}  // namespace apollo
