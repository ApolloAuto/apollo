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

#include <poll.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <iostream>

#include "gflags/gflags.h"
#include "ros/include/ros/ros.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/hdmap/hdmap_util.h"

using apollo::common::PointENU;
using apollo::hdmap::HDMap;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;
using apollo::common::util::PrintIter;

DEFINE_double(traffic_light_distance, 1000.0,
              "only retrieves traffic lights within this distance");

bool g_is_green = false;
bool g_updated = true;

constexpr char RED_COLOR[] = "\033[41m";
constexpr char GREEN_COLOR[] = "\033[42m";
constexpr char RESET_COLOR[] = "\033[0m";

bool GetTrafficLights(std::vector<SignalInfoConstPtr> *traffic_lights) {
  CHECK_NOTNULL(traffic_lights);
  AdapterManager::Observe();
  if (AdapterManager::GetLocalization()->Empty()) {
    ADEBUG << "No localization received";
    return false;
  }
  const auto *hdmap = HDMapUtil::BaseMapPtr();
  auto position =
      AdapterManager::GetLocalization()->GetLatestObserved().pose().position();
  int ret = hdmap->GetForwardNearestSignalsOnLane(
      position, FLAGS_traffic_light_distance, traffic_lights);
  if (ret != 0) {
    AERROR << "failed to get signals from position: "
           << position.ShortDebugString()
           << " with distance: " << FLAGS_traffic_light_distance << " on map";
    return false;
  }
  return true;
}

bool CreateTrafficLightDetection(const std::vector<SignalInfoConstPtr> &signals,
                                 TrafficLight::Color color,
                                 TrafficLightDetection *detection) {
  CHECK_NOTNULL(detection);
  for (const auto &iter : signals) {
    auto *light = detection->add_traffic_light();
    light->set_color(color);
    light->set_confidence(1.0);
    light->set_tracking_time(1.0);
    light->set_id(iter->id().id());
  }
  AdapterManager::FillTrafficLightDetectionHeader("manual_traffic_light",
                                                  detection);
  return true;
}

TrafficLight::Color GetKeyBoardColorInput() {
  int8_t revent = 0;  // short
  struct pollfd fd = {STDIN_FILENO, POLLIN, revent};
  switch (poll(&fd, 1, 100)) {
    case -1:
      AERROR << "Failed to read keybapoard";
      break;
    case 0:
      break;
    default:
      char ch = 'x';
      std::cin >> ch;
      if (ch == 'c') {
        g_is_green = !g_is_green;
        g_updated = true;
      }
      break;
  }
  if (g_is_green) {
    return TrafficLight::GREEN;
  } else {
    return TrafficLight::RED;
  }
}

bool IsDifferent(const std::unordered_set<std::string> &str_set,
                 const std::vector<std::string> &str_vec) {
  if (str_vec.size() != str_set.size()) {
    return true;
  }
  for (const auto &ss : str_vec) {
    if (str_set.count(ss) == 0) {
      return true;
    }
  }
  return false;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "manual_traffic_light");

  AdapterManagerConfig manager_config;
  auto *config = manager_config.add_config();
  config->set_type(AdapterConfig::TRAFFIC_LIGHT_DETECTION);
  config->set_mode(AdapterConfig::PUBLISH_ONLY);
  config->set_message_history_limit(1);

  config = manager_config.add_config();
  config->set_type(AdapterConfig::LOCALIZATION);
  config->set_mode(AdapterConfig::RECEIVE_ONLY);
  config->set_message_history_limit(1);
  manager_config.set_is_ros(true);
  AdapterManager::Init(manager_config);
  std::unordered_set<std::string> prev_traffic_lights;

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    std::vector<SignalInfoConstPtr> signals;
    if (!GetTrafficLights(&signals)) {
      ADEBUG << "Failed to get traffic signals from current location on map";
      continue;
    } else {
      ADEBUG << "Received " << signals.size() << " traffic lights";
    }
    std::vector<std::string> signal_ids;
    for (const auto &ptr : signals) {
      signal_ids.emplace_back(ptr->id().id());
    }
    if (IsDifferent(prev_traffic_lights, signal_ids)) {
      prev_traffic_lights =
          std::unordered_set<std::string>(signal_ids.begin(), signal_ids.end());
      g_updated = true;
    }
    TrafficLightDetection traffic_light_detection;
    TrafficLight::Color color = GetKeyBoardColorInput();
    ADEBUG << "Color: " << TrafficLight::Color_Name(color);
    if (g_updated) {
      g_updated = false;
      const char *print_color = g_is_green ? GREEN_COLOR : RED_COLOR;
      std::cout << print_color
                << "Current Light: " << (g_is_green ? "GREEN" : "RED");
      if (signal_ids.empty()) {
        std::cout << " No lights in the next " << FLAGS_traffic_light_distance
                  << " meters";
      } else {
        std::cout << " IDs: "
                  << PrintIter(signal_ids.begin(), signal_ids.end());
      }
      std::cout << std::endl
                << RESET_COLOR << "Press 'c' to change" << std::endl
                << std::endl;
    }
    CreateTrafficLightDetection(signals, color, &traffic_light_detection);
    AdapterManager::PublishTrafficLightDetection(traffic_light_detection);
  }
  return 0;
}
