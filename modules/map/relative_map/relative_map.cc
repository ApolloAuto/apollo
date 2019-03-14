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

#include <yaml-cpp/yaml.h>
#include "third_party/json/json.hpp"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace apollo {
namespace relative_map {

using nlohmann::json;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::operator+;
using apollo::common::math::Vec2d;
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

  config_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_relative_map_config_filename,
                                      &config_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_relative_map_config_filename);
  }

  navigation_lane_.SetConfig(config_.navigation_lane());
  const auto& map_param = config_.map_param();
  navigation_lane_.SetDefaultWidth(map_param.default_left_width(),
                                   map_param.default_right_width());

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

void LogErrorStatus(MapMsg* map_msg, const std::string& error_msg) {
  auto* status = map_msg->mutable_header()->mutable_status();
  status->set_msg(error_msg);
  status->set_error_code(ErrorCode::RELATIVE_MAP_ERROR);
}

apollo::common::Status RelativeMap::Start() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap started");

  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_relative_map_loop_rate), &RelativeMap::OnTimer,
      this);

  // donot receive topic navigation while 
  // FLAGS_load_navigation_path_when_start is true
  if (!FLAGS_load_navigation_path_when_start) {
    AdapterManager::AddNavigationCallback(&RelativeMap::OnReceiveNavigationInfo,
                                        this);
  }

  if (AdapterManager::GetPerceptionObstacles()->Empty()) {
    AWARN << "Perception is not ready.";
  }

  // start to load navigation path
  if (FLAGS_load_navigation_path_when_start) {
    LoadNavigationPath();
  }

  return Status::OK();
}

void RelativeMap::OnTimer(const ros::TimerEvent&) { RunOnce(); }

void RelativeMap::RunOnce() {
  AdapterManager::Observe();

  MapMsg map_msg;
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    CreateMapFromNavigationLane(&map_msg);
  }
  Publish(&map_msg);
}

void RelativeMap::OnReceiveNavigationInfo(
    const NavigationInfo& navigation_info) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    navigation_lane_.UpdateNavigationInfo(navigation_info);
  }
}

bool RelativeMap::CreateMapFromNavigationLane(MapMsg* map_msg) {
  CHECK_NOTNULL(map_msg);

  if (AdapterManager::GetLocalization()->Empty()) {
    LogErrorStatus(map_msg, "localization is not ready");
    return false;
  }
  if (AdapterManager::GetChassis()->Empty()) {
    LogErrorStatus(map_msg, "chassis is not ready");
    return false;
  }

  // update vehicle state from localization and chassis
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();
  VehicleStateProvider::instance()->Update(localization, chassis);
  map_msg->mutable_localization()->CopyFrom(localization);

  // update navigation_lane from perception_obstacles (lane marker)
  if (!AdapterManager::GetPerceptionObstacles()->Empty()) {
    const auto& perception =
        AdapterManager::GetPerceptionObstacles()->GetLatestObserved();
    navigation_lane_.UpdatePerception(perception);
    map_msg->mutable_lane_marker()->CopyFrom(perception.lane_marker());
  }

  if (!navigation_lane_.GeneratePath()) {
    LogErrorStatus(map_msg, "Failed to generate a navigation path.");
    return false;
  }

  if (navigation_lane_.Path().path().path_point_size() == 0) {
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

bool RelativeMap::GetNavigationPathList(
                  std::vector<std::string>* const path_list) {
  if (!path_list) {
    return false;
  }

  YAML::Node path_root = YAML::LoadFile(
      FLAGS_relative_map_navigation_path_filename);
  if (!path_root["navigation_path"]) {
    return false;
  }

  const auto& path = path_root["navigation_path"];
  for (std::size_t i = 0; i < path.size(); ++i) {
    path_list->push_back(path[i]["name"].as<std::string>());
  }
  return true;
}

void RelativeMap::LoadNavigationPath() {
  if (!FLAGS_load_navigation_path_when_start) {
    AERROR << "it's disabled to load navigation path when start";
    return;
  }

  std::vector<std::string> path_list;
  bool ret = GetNavigationPathList(&path_list);
  if (!ret) {
    AERROR << "Failed to load navigation path list";
    return;
  }

  NavigationInfo navigation_info;
  int priority = 0;
  for (const auto& path_file : path_list) {
    AINFO << "begin to load navigation path " << path_file;
    std::ifstream fstr(path_file, std::ios::in);
    if (!fstr.is_open()) {
      AERROR << "Failed to open navigation path file " 
             << FLAGS_relative_map_navigation_path_filename;
      return;
    }

    std::string line_con;
    auto* navigation_path = navigation_info.add_navigation_path();
    navigation_path->set_path_priority(priority);
    auto* path = navigation_path->mutable_path();
    path->set_name("navigation");

    try {
      while (std::getline(fstr, line_con)) {
        auto json_node = json::parse(line_con);
        auto* point = path->add_path_point();
        point->set_x(json_node["x"]);
        point->set_y(json_node["y"]);
        point->set_s(json_node["s"]);
        point->set_theta(json_node["theta"]);
        point->set_kappa(json_node["kappa"]);
        point->set_dkappa(json_node["dkappa"]);
      } 
    } catch (const std::exception& e) {
        AERROR << "Failed to load navigation path file, catch exception "
               << e.what();
        fstr.close();
        return;
    }

    fstr.close();
    priority++;
  }

   PublishNavigationInfo(&navigation_info);

   navigation_lane_.UpdateNavigationInfo(std::move(navigation_info));
}

void RelativeMap::Stop() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap stopped");
}

}  // namespace relative_map
}  // namespace apollo
