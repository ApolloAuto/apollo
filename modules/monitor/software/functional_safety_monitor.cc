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

#include "modules/monitor/software/functional_safety_monitor.h"

#include <string>

#include "absl/strings/str_cat.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(functional_safety_monitor_name, "FunctionalSafetyMonitor",
              "Name of the functional safety monitor.");

DEFINE_double(safety_mode_seconds_before_estop, 10.0,
              "Interval before sending estop after we found critical errors.");

namespace apollo {
namespace monitor {
namespace {

bool IsSafe(const std::string& name, const ComponentStatus& status) {
  if (status.status() == ComponentStatus::ERROR ||
      status.status() == ComponentStatus::FATAL) {
    MonitorManager::Instance()->LogBuffer().ERROR(
        absl::StrCat(name, " triggers safe mode: ", status.message()));
    return false;
  }
  return true;
}

}  // namespace

FunctionalSafetyMonitor::FunctionalSafetyMonitor()
    : RecurrentRunner(FLAGS_functional_safety_monitor_name, 0) {}

void FunctionalSafetyMonitor::RunOnce(const double current_time) {
  auto* system_status = MonitorManager::Instance()->GetStatus();
  // Everything looks good or has been handled properly.
  if (CheckSafety()) {
    system_status->clear_passenger_msg();
    system_status->clear_safety_mode_trigger_time();
    system_status->clear_require_emergency_stop();
    return;
  }
  if (system_status->require_emergency_stop()) {
    // EStop has already been triggered.
    return;
  }

  // Newly entered safety mode.
  system_status->set_passenger_msg("Error! Please disengage.");
  if (!system_status->has_safety_mode_trigger_time()) {
    system_status->set_safety_mode_trigger_time(current_time);
    return;
  }

  // Trigger EStop if no action was taken in time.
  if (system_status->safety_mode_trigger_time() +
          FLAGS_safety_mode_seconds_before_estop <
      current_time) {
    system_status->set_require_emergency_stop(true);
  }
}

bool FunctionalSafetyMonitor::CheckSafety() {
  // We only check safety in self driving mode.
  auto manager = MonitorManager::Instance();
  if (!manager->IsInAutonomousMode()) {
    return true;
  }

  // Check HMI modules status.
  const auto& mode = manager->GetHMIMode();
  const auto& hmi_modules = manager->GetStatus()->hmi_modules();
  for (const auto& iter : mode.modules()) {
    const std::string& module_name = iter.first;
    const auto& module = iter.second;
    if (module.required_for_safety() &&
        !IsSafe(module_name, hmi_modules.at(module_name))) {
      return false;
    }
  }

  // Check monitored components status.
  const auto& components = manager->GetStatus()->components();
  for (const auto& iter : mode.monitored_components()) {
    const std::string& component_name = iter.first;
    const auto& component = iter.second;
    if (component.required_for_safety() &&
        !IsSafe(component_name, components.at(component_name).summary())) {
      return false;
    }
  }

  // Everything looks good.
  return true;
}

}  // namespace monitor
}  // namespace apollo
