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

#include "cyber/common/file.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::monitor::MonitorMessageItem;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;

RelativeMap::RelativeMap()
    : monitor_logger_buffer_(MonitorMessageItem::RELATIVE_MAP),
      vehicle_state_provider_(nullptr) {}

Status RelativeMap::Init(common::VehicleStateProvider* vehicle_state_provider) {
  vehicle_state_provider_ = vehicle_state_provider;
  if (!FLAGS_use_navigation_mode) {
    AERROR << "FLAGS_use_navigation_mode is false, system is not configured "
              "for relative map mode";
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "FLAGS_use_navigation_mode is not true.");
  }
  config_.Clear();
  if (!cyber::common::GetProtoFromFile(FLAGS_relative_map_config_filename,
                                       &config_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_relative_map_config_filename);
  }

  navigation_lane_.SetConfig(config_.navigation_lane());
  navigation_lane_.SetVehicleStateProvider(vehicle_state_provider);
  const auto& map_param = config_.map_param();
  navigation_lane_.SetDefaultWidth(map_param.default_left_width(),
                                   map_param.default_right_width());

  return Status::OK();
}

void LogErrorStatus(MapMsg* map_msg, const std::string& error_msg) {
  auto* status = map_msg->mutable_header()->mutable_status();
  status->set_msg(error_msg);
  status->set_error_code(ErrorCode::RELATIVE_MAP_ERROR);
}

apollo::common::Status RelativeMap::Start() {
  monitor_logger_buffer_.INFO("RelativeMap started");
  return Status::OK();
}

bool RelativeMap::Process(MapMsg* const map_msg) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    CreateMapFromNavigationLane(map_msg);
  }
  return true;
}

void RelativeMap::OnNavigationInfo(const NavigationInfo& navigation_info) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    navigation_lane_.UpdateNavigationInfo(navigation_info);
  }
}

void RelativeMap::OnPerception(
    const PerceptionObstacles& perception_obstacles) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    perception_obstacles_.CopyFrom(perception_obstacles);
  }
}

void RelativeMap::OnChassis(const Chassis& chassis) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    chassis_.CopyFrom(chassis);
  }
}

void RelativeMap::OnLocalization(const LocalizationEstimate& localization) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    localization_.CopyFrom(localization);
  }
}

bool RelativeMap::CreateMapFromNavigationLane(MapMsg* map_msg) {
  CHECK_NOTNULL(map_msg);

  // update vehicle state from localization and chassis

  LocalizationEstimate const& localization = localization_;
  Chassis const& chassis = chassis_;
  vehicle_state_provider_->Update(localization, chassis);
  map_msg->mutable_localization()->CopyFrom(localization_);

  // update navigation_lane from perception_obstacles (lane marker)
  PerceptionObstacles const& perception = perception_obstacles_;
  navigation_lane_.UpdatePerception(perception);
  map_msg->mutable_lane_marker()->CopyFrom(perception_obstacles_.lane_marker());

  if (!navigation_lane_.GeneratePath()) {
    LogErrorStatus(map_msg, "Failed to generate a navigation path.");
    return false;
  }

  if (navigation_lane_.Path().path().path_point().empty()) {
    LogErrorStatus(map_msg,
                   "There is no path point in currnet navigation path.");
    return false;
  }

  // create map proto from navigation_path
  if (!navigation_lane_.CreateMap(config_.map_param(), map_msg)) {
    LogErrorStatus(map_msg,
                   "Failed to create map from current navigation path.");
    AERROR << "Failed to create map from navigation path.";
    return false;
  }

  ADEBUG << "There is/are " << map_msg->navigation_path().size()
         << " navigation path(s) in the current reltative map.";
  return true;
}

void RelativeMap::Stop() { monitor_logger_buffer_.INFO("RelativeMap stopped"); }

}  // namespace relative_map
}  // namespace apollo
