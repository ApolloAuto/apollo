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

#include "cybertron/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(localization_monitor_name, "LocalizationMonitor",
              "Name of the localization monitor.");

DEFINE_double(localization_monitor_interval, 5,
              "Localization status checking interval (s).");

DEFINE_string(localization_module_name, "localization",
              "Localization module name.");

namespace apollo {
namespace monitor {
using apollo::common::util::StrCat;
using apollo::localization::LocalizationStatus;
using apollo::localization::MeasureState;
using apollo::localization::MeasureState_Name;

LocalizationMonitor::LocalizationMonitor()
    : RecurrentRunner(FLAGS_localization_monitor_name,
                      FLAGS_localization_monitor_interval) {}

void LocalizationMonitor::RunOnce(const double current_time) {
  static auto reader = MonitorManager::CreateReader<LocalizationStatus>(
      FLAGS_localization_msf_status);
  reader->Observe();
  const auto status = reader->GetLatestObserved();
  if (status == nullptr) {
    AERROR << "No LocalizationStatus received.";
    return;
  }

  auto& localization_status = apollo::common::util::LookupOrInsert(
      MonitorManager::GetStatus()->mutable_modules(),
      FLAGS_localization_module_name, {});
  auto& dv_log = MonitorManager::LogBuffer();

  switch (status->fusion_status()) {
    case MeasureState::OK:
      localization_status.set_summary(Summary::OK);
      break;
    case MeasureState::WARNNING:
      AWARN << "Localization WARNNING: " << status->state_message();
      localization_status.set_summary(Summary::WARN);
      break;
    case MeasureState::ERROR:
      dv_log.WARN(StrCat("Localization ERROR: ", status->state_message()));
      localization_status.set_summary(Summary::WARN);
      break;
    case MeasureState::CRITICAL_ERROR:
      dv_log.ERROR(StrCat(
          "Localization CRITICAL_ERROR: ", status->state_message()));
      localization_status.set_summary(Summary::WARN);
      break;
    case MeasureState::FATAL_ERROR:
      dv_log.ERROR(StrCat(
          "Localization FATAL_ERROR: ", status->state_message()));
      // ERROR and FATAL will trigger safety stop.
      localization_status.set_summary(Summary::FATAL);
      break;
    default:
      AFATAL << "Unknown fusion_status: " << status->fusion_status();
      break;
  }
  localization_status.set_msg(status->state_message());
}

}  // namespace monitor
}  // namespace apollo
