/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/localization_monitor.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"

DEFINE_string(localization_monitor_name, "LocalizationMonitor",
              "Name of the localization monitor.");

DEFINE_double(localization_monitor_interval, 5,
              "Localization status checking interval (s).");

namespace apollo {
namespace monitor {
using apollo::common::adapter::AdapterManager;
using apollo::localization::MeasureState;
using apollo::localization::MeasureState_Name;

LocalizationMonitor::LocalizationMonitor()
    : RecurrentRunner(FLAGS_localization_monitor_name,
                      FLAGS_localization_monitor_interval) {
  CHECK_NOTNULL(AdapterManager::GetLocalizationMsfStatus());
}

void LocalizationMonitor::RunOnce(const double current_time) {
  auto* adapter = AdapterManager::GetLocalizationMsfStatus();
  adapter->Observe();
  if (adapter->Empty()) {
    AERROR << "No LocalizationStatus received.";
    return;
  }

  // TODO(xiaoxq): Decide situations of escalating warnings to:
  //   1. Log to Dreamview as warning.
  //   2. Log to Dreamview as error.
  //   3. Read aloud to passengers.
  //   4. Trigger guardian safety stop.
  const auto& status = adapter->GetLatestObserved();
  if (status.gnss_status() != MeasureState::OK) {
    AWARN << "Localization GNSS status "
          << MeasureState_Name(status.gnss_status());
  }
  if (status.lidar_status() != MeasureState::OK) {
    AWARN << "Localization lidar status "
          << MeasureState_Name(status.lidar_status());
  }
  if (status.fusion_status() != MeasureState::OK) {
    AERROR << "Localization fusion status "
           << MeasureState_Name(status.fusion_status());
  }
}

}  // namespace monitor
}  // namespace apollo
