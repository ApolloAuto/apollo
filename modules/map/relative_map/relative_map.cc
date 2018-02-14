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

#include "modules/map/relative_map/relative_map.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorLogBuffer;
using apollo::common::monitor::MonitorMessageItem;
using apollo::perception::PerceptionObstacles;
using apollo::common::VehicleStateProvider;

RelativeMap::RelativeMap()
    : monitor_logger_(MonitorMessageItem::RELATIVE_MAP) {}

Status RelativeMap::Init() {
  if (!FLAGS_use_navigation_mode) {
    AERROR << "FLAGS_use_navigation_mode is false, system is not configured "
              "for relative map mode";
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "FLAGS_use_navigation_mode is not true.");
  }
  adapter_conf_.Clear();
  if (!common::util::GetProtoFromFile(
          FLAGS_relative_map_adapter_config_filename, &adapter_conf_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load adapter conf file: " +
                      FLAGS_relative_map_adapter_config_filename);
  } else {
    ADEBUG << "Adapter config file is loaded into: "
           << adapter_conf_.ShortDebugString();
  }

  AdapterManager::Init(adapter_conf_);
  if (!AdapterManager::GetPerceptionObstacles()) {
    std::string error_msg(
        "Perception should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (!AdapterManager::GetMonitor()) {
    std::string error_msg(
        "Monitor should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (AdapterManager::GetLocalization() == nullptr) {
    std::string error_msg(
        "Localization should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (AdapterManager::GetChassis() == nullptr) {
    std::string error_msg(
        "Chassis should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (AdapterManager::GetNavigation() == nullptr) {
    std::string error_msg(
        "Navigation should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  return Status::OK();
}

apollo::common::Status RelativeMap::Start() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap started");

  AdapterManager::AddPerceptionObstaclesCallback(&RelativeMap::RunOnce, this);
  AdapterManager::AddNavigationCallback(&RelativeMap::RunOnce, this);

  if (AdapterManager::GetPerceptionObstacles()->Empty()) {
    AWARN << "Perception is not ready.";
  }

  return Status::OK();
}

void RelativeMap::RunOnce(const PerceptionObstacles& perception_obstacles) {
  AdapterManager::Observe();

  MapMsg map_msg;
  CreateMapFromPerception(perception_obstacles, &map_msg);
  Publish(&map_msg);
}

void RelativeMap::RunOnce(const NavigationInfo& navigation_info) {
  AdapterManager::Observe();
  navigation_lane_.UpdateNavigationInfo(navigation_info);
}

void RelativeMap::CreateMapFromPerception(
    const PerceptionObstacles& perception_obstacles, MapMsg* map_msg) {
  // update vehicle state from localization and chassis
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();
  VehicleStateProvider::instance()->Update(localization, chassis);

  // update navigation_lane from perception_obstacles (lane marker)
  navigation_lane_.Update(perception_obstacles);

  const auto& navigation_path = navigation_lane_.Path();
  // TODO(all) create map proto from navigation_path
}

void RelativeMap::Stop() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap stopped");
}

}  // namespace relative_map
}  // namespace apollo
