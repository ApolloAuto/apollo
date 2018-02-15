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

#include "modules/map/proto/map_lane.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::operator+;
using apollo::common::math::Vec2d;
using apollo::common::monitor::MonitorLogBuffer;
using apollo::common::monitor::MonitorMessageItem;
using apollo::hdmap::Lane;
// using apollo::common::util::operator+;
using apollo::hdmap::LaneBoundaryType;
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

  config_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_relative_map_config_filename,
                                      &config_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_relative_map_config_filename);
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

  ADEBUG << "PerceptionObstacles received by RelativeMap:\n"
         << perception_obstacles.DebugString();

  MapMsg map_msg;
  CreateMapFromPerception(perception_obstacles, &map_msg);
  if (map_msg.has_hdmap()) {
    Publish(&map_msg);
  }
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

  // create map proto from navigation_path
  if (!CreateMapFromNavigationPath(
          navigation_lane_.Path(), navigation_lane_.left_width(),
          navigation_lane_.right_width(), map_msg->mutable_hdmap())) {
    map_msg->clear_hdmap();
    AERROR << "Failed to create map from navigation path";
  }
}

bool RelativeMap::CreateMapFromNavigationPath(
    const NavigationPath& navigation_path, double left_width,
    double right_width, hdmap::Map* hdmap) {
  const auto& path = navigation_path.path();
  if (path.path_point_size() < 2) {
    AERROR << "The path length is invalid";
    return false;
  }
  const auto& map_config = config_.map_param();
  auto* lane = hdmap->add_lane();
  lane->mutable_id()->set_id(std::to_string(navigation_path.path_priority()) +
                             "_" + path.name());
  // lane types
  lane->set_type(Lane::CITY_DRIVING);
  lane->set_turn(Lane::NO_TURN);

  // speed limit
  lane->set_speed_limit(map_config.default_speed_limit());

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  auto* line_segment = curve_segment->mutable_line_segment();
  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(LaneBoundaryType::SOLID_YELLOW);
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(LaneBoundaryType::SOLID_YELLOW);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();
  const double lane_left_width =
      left_width > 0 ? left_width : map_config.default_left_width();
  const double lane_right_width =
      right_width > 0 ? right_width : map_config.default_right_width();
  for (const auto& path_point : path.path_point()) {
    auto* point = line_segment->add_point();
    point->set_x(path_point.x());
    point->set_y(path_point.y());
    point->set_z(path_point.z());
    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(path_point.s());
    left_sample->set_width(lane_left_width);
    left_segment->add_point()->CopyFrom(
        *point +
        lane_left_width * Vec2d::CreateUnitVec2d(path_point.theta() + M_PI_2));
    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path_point.s());
    right_sample->set_width(lane_right_width);
    right_segment->add_point()->CopyFrom(
        *point +
        lane_right_width * Vec2d::CreateUnitVec2d(path_point.theta() - M_PI_2));
  }

  return true;
}

void RelativeMap::Stop() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap stopped");
}

}  // namespace relative_map
}  // namespace apollo
