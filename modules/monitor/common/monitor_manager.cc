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

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/util/hmi_util.h"

namespace apollo {
namespace monitor {

using apollo::canbus::Chassis;

MonitorManager::MonitorManager()
    : hmi_config_(apollo::dreamview::util::HMIUtil::LoadConfig()),
      log_buffer_(apollo::common::monitor::MonitorMessageItem::MONITOR) {}

void MonitorManager::Init(const std::shared_ptr<apollo::cyber::Node>& node) {
  node_ = node;
  if (FLAGS_use_sim_time) {
    status_.set_is_realtime_in_simulation(true);
  }
}

bool MonitorManager::StartFrame(const double current_time) {
  status_.set_detect_immediately(false);
  // Get latest HMIStatus.
  static auto hmi_status_reader =
      CreateReader<apollo::dreamview::HMIStatus>(FLAGS_hmi_status_topic);
  hmi_status_reader->Observe();
  const auto hmi_status = hmi_status_reader->GetLatestObserved();
  if (hmi_status == nullptr) {
    AERROR << "No HMIStatus was received.";
    return false;
  } else if (hmi_status->expected_modules() > 0) {
    status_.set_detect_immediately(true);
  }

  if (current_mode_ != hmi_status->current_mode()) {
    // Mode changed, update configs and monitored.
    current_mode_ = hmi_status->current_mode();
    mode_config_ = apollo::dreamview::util::HMIUtil::LoadMode(
        hmi_config_.modes().at(current_mode_));
    status_.clear_hmi_modules();
    for (const auto& iter : mode_config_.modules()) {
      status_.mutable_hmi_modules()->insert({iter.first, {}});
    }
    status_.clear_components();
    for (const auto& iter : mode_config_.monitored_components()) {
      status_.mutable_components()->insert({iter.first, {}});
    }
    status_.clear_other_components();
    for (const auto& iter : mode_config_.other_components()) {
      status_.mutable_other_components()->insert({iter.first, {}});
    }
    status_.clear_global_components();
    for (const auto& iter : mode_config_.global_components()) {
      status_.mutable_global_components()->insert({iter.first, {}});
    }
  } else {
    // Mode not changed, clear component summary from the last frame.
    for (auto& iter : *status_.mutable_components()) {
      iter.second.clear_summary();
    }
    // clear global component summary from the last frame.
    for (auto& iter : *status_.mutable_global_components()) {
      iter.second.clear_summary();
    }
  }

  in_autonomous_driving_ = CheckAutonomousDriving(current_time);
  return true;
}

void MonitorManager::EndFrame() {
  // Print and publish all monitor logs.
  log_buffer_.Publish();
}

bool MonitorManager::CheckAutonomousDriving(const double current_time) {
  // It's in offline mode if use_sim_time is set.
  if (FLAGS_use_sim_time) {
    return false;
  }

  // Get current DrivingMode, which will affect how we monitor modules.
  static auto chassis_reader = CreateReader<Chassis>(FLAGS_chassis_topic);
  chassis_reader->Observe();
  const auto chassis = chassis_reader->GetLatestObserved();
  if (chassis == nullptr) {
    return false;
  }

  // If SimControl is used, return false to ignore safety check.
  if (chassis->header().module_name() == "SimControl") {
    return false;
  }

  // Ignore old messages which are likely from playback.
  const double msg_time = chassis->header().timestamp_sec();
  if (msg_time + FLAGS_system_status_lifetime_seconds < current_time) {
    return false;
  }

  return chassis->driving_mode() == Chassis::COMPLETE_AUTO_DRIVE;
}

}  // namespace monitor
}  // namespace apollo
