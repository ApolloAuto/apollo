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

#include "modules/monitor/software/safety_manager.h"

#include "modules/common/kv_db/kv_db.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/string_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_double(safety_mode_seconds_before_estop, 10.0,
              "Interval before sending estop after we found critical errors.");

namespace apollo {
namespace monitor {

using apollo::common::KVDB;
using apollo::common::util::ContainsKey;
using apollo::common::util::GetProtoFromFile;
using apollo::common::util::FindOrNull;
using apollo::common::util::StrCat;

SafetyManager::SafetyManager() {
  CHECK(GetProtoFromFile(FLAGS_hmi_config_filename, &hmi_config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;
}

void SafetyManager::CheckSafety(const double current_time) {
  auto *system_status = MonitorManager::GetStatus();
  // Everything looks good or has been handled properly.
  if (!ShouldTriggerSafeMode(current_time)) {
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
      FLAGS_safety_mode_seconds_before_estop < current_time) {
    system_status->set_require_emergency_stop(true);
  }
}

bool SafetyManager::ShouldTriggerSafeMode(const double current_time) {
  // We only check safety mode in self driving mode.
  if (!MonitorManager::IsInAutonomousDriving()) {
    return false;
  }

  const std::string mode_name = KVDB::Get("apollo:dreamview:mode");
  auto& log = MonitorManager::LogBuffer();
  if (mode_name.empty()) {
    log.ERROR("Cannot get apollo mode");
    return true;
  }

  const apollo::dreamview::Mode *mode_conf =
      FindOrNull(hmi_config_.modes(), mode_name);
  if (mode_conf == nullptr) {
    log.ERROR(StrCat("Cannot find configuration for apollo mode: ", mode_name));
    return true;
  }

  const auto &hardware_status = MonitorManager::GetStatus()->hardware();
  for (const auto &hardware : mode_conf->live_hardware()) {
    const auto *status = FindOrNull(hardware_status, hardware);
    if (status == nullptr) {
      log.ERROR(StrCat("Cannot get status of hardware: ", hardware));
      return true;
    }
    if (status->summary() == Summary::ERROR ||
        status->summary() == Summary::FATAL) {
      log.ERROR(StrCat(
          "Hardware ", hardware, " triggers safety mode: ", status->msg()));
      return true;
    }
  }

  const auto& modules_status = MonitorManager::GetStatus()->modules();
  for (const auto &module : mode_conf->live_modules()) {
    const auto *status = FindOrNull(modules_status, module);
    if (status == nullptr) {
      log.ERROR(StrCat("Cannot get status of module: ", module));
      return true;
    }
    if (status->summary() == Summary::ERROR ||
        status->summary() == Summary::FATAL) {
      log.ERROR(StrCat(
          "Module ", module, " triggers safety mode: ", status->msg()));
      return true;
    }
  }

  // TODO(xiaoxq): We may also need to trigger safety mode if GPS becomes very
  // unstable.
  return false;
}

}  // namespace monitor
}  // namespace apollo
