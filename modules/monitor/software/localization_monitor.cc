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

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(localization_monitor_name, "LocalizationMonitor",
              "Name of the localization monitor.");

DEFINE_double(localization_monitor_interval, 5,
              "Localization status checking interval (s).");

DEFINE_string(localization_component_name, "Localization",
              "Localization component name.");

namespace apollo {
namespace monitor {
using apollo::localization::LocalizationStatus;
using apollo::localization::MeasureState;

LocalizationMonitor::LocalizationMonitor()
    : RecurrentRunner(FLAGS_localization_monitor_name,
                      FLAGS_localization_monitor_interval) {}

void LocalizationMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  auto* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(),
      FLAGS_localization_component_name);
  if (component == nullptr) {
    // localization is not monitored in current mode, skip.
    return;
  }

  static auto reader =
      manager->CreateReader<LocalizationStatus>(FLAGS_localization_msf_status);
  reader->Observe();
  const auto status = reader->GetLatestObserved();

  ComponentStatus* component_status = component->mutable_other_status();
  component_status->clear_status();
  if (status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                   "No LocalizationStatus received",
                                   component_status);
    return;
  }

  // Translate LocalizationStatus to ComponentStatus. Note that ERROR and FATAL
  // will trigger safety mode in current settings.
  switch (status->fusion_status()) {
    case MeasureState::OK:
      SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", component_status);
      break;
    case MeasureState::WARNNING:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN,
          absl::StrCat("WARNNING: ", status->state_message()),
          component_status);
      break;
    case MeasureState::ERROR:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN,
          absl::StrCat("ERROR: ", status->state_message()), component_status);
      break;
    case MeasureState::CRITICAL_ERROR:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::ERROR,
          absl::StrCat("CRITICAL_ERROR: ", status->state_message()),
          component_status);
      break;
    case MeasureState::FATAL_ERROR:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::FATAL,
          absl::StrCat("FATAL_ERROR: ", status->state_message()),
          component_status);
      break;
    default:
      AFATAL << "Unknown fusion_status: " << status->fusion_status();
      break;
  }
}

}  // namespace monitor
}  // namespace apollo
