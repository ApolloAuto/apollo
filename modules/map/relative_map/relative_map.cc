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
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorLogBuffer;
using apollo::common::monitor::MonitorMessageItem;
using apollo::perception::PerceptionObstacles;

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
    AERROR << "Perception should be configured as dependency in adapter.conf";
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "perception is not configured");
  }
  if (!AdapterManager::GetMonitor()) {
    AERROR << "Monitor should be configured as dependency in adapter.conf";
    return Status(ErrorCode::RELATIVE_MAP_ERROR, "Monitor is not configured");
  }
  return Status::OK();
}

apollo::common::Status RelativeMap::Start() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap started");

  AdapterManager::AddPerceptionObstaclesCallback(&RelativeMap::RunOnce, this);

  if (AdapterManager::GetPerceptionObstacles()->Empty()) {
    AWARN << "Perception is not ready.";
  }

  return Status::OK();
}

void RelativeMap::RunOnce(const PerceptionObstacles& perception_obstacles) {
  MapMsg map_msg;
  CreateMapFromPerception(perception_obstacles, &map_msg);
  Publish(&map_msg);
}

void RelativeMap::CreateMapFromPerception(
    const PerceptionObstacles& perception_obstacles, MapMsg* map_msg) {
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
