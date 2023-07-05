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

#include "absl/strings/match.h"
#include "absl/strings/str_join.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/color.h"
#include "modules/common/util/message_util.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"

using apollo::common::color::ANSI_GREEN;
using apollo::common::color::ANSI_RED;
using apollo::common::color::ANSI_RESET;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

DEFINE_bool(all_lights, false, "set all lights on the map");

DEFINE_double(traffic_light_distance, 1000.0,
              "only retrieves traffic lights within this distance");

class ManualTrafficLight final : public apollo::cyber::TimerComponent {
 public:
  bool Init() {
    localization_reader_ = node_->CreateReader<LocalizationEstimate>(
        FLAGS_localization_topic,
        [this](const std::shared_ptr<LocalizationEstimate> &localization) {
          ADEBUG << "Received chassis data: run chassis callback.";
          OnLocalization(localization);
        });

    traffic_light_detection_writer_ =
        node_->CreateWriter<TrafficLightDetection>(
            FLAGS_traffic_light_detection_topic);

    return true;
  }

  bool Proc() {
    std::vector<SignalInfoConstPtr> signals;
    bool result = false;
    if (FLAGS_all_lights) {
      result = GetAllTrafficLights(&signals);
    } else {
      result = GetTrafficLightsWithinDistance(&signals);
    }
    if (!result) {
      ADEBUG << "Failed to get traffic signals from current location on map";
    } else {
      ADEBUG << "Received " << signals.size() << " traffic lights";
    }
    std::vector<std::string> signal_ids;
    for (const auto &ptr : signals) {
      signal_ids.emplace_back(ptr->id().id());
    }
    if (IsDifferent(prev_traffic_lights_, signal_ids)) {
      prev_traffic_lights_ =
          std::unordered_set<std::string>(signal_ids.begin(), signal_ids.end());
      updated_ = true;
    }
    TrafficLightDetection traffic_light_detection;
    TrafficLight::Color color = GetKeyBoardColorInput();
    ADEBUG << "Color: " << TrafficLight::Color_Name(color);
    if (updated_) {
      updated_ = false;
      const char *print_color = is_green_ ? ANSI_GREEN : ANSI_RED;
      std::cout << print_color
                << "Current Light: " << (is_green_ ? "GREEN" : "RED");
      if (signal_ids.empty()) {
        if (FLAGS_all_lights) {
          std::cout << " No lights in the map";
        } else {
          std::cout << " No lights in the next " << FLAGS_traffic_light_distance
                    << " meters";
        }
      } else {
        if (signal_ids.size() < 5) {
          std::cout << " IDs: " << absl::StrJoin(signal_ids, " ");
        } else {
          std::cout << " IDs: "
                    << absl::StrJoin(signal_ids.begin(), signal_ids.begin() + 4,
                                     " ")
                    << " ...";
        }
      }
      std::cout << std::endl
                << ANSI_RESET << "Press 'c' to change" << std::endl
                << std::endl;
    }
    CreateTrafficLightDetection(signals, color, &traffic_light_detection);
    traffic_light_detection_writer_->Write(traffic_light_detection);
    return true;
  }

 private:
  bool GetAllTrafficLights(std::vector<SignalInfoConstPtr> *traffic_lights) {
    static auto map_filename = apollo::hdmap::BaseMapFile();
    static apollo::hdmap::Map map_proto;
    static std::vector<SignalInfoConstPtr> map_traffic_lights;
    if (map_proto.lane().empty() && map_traffic_lights.empty()) {
      AERROR << "signal size: " << map_proto.signal_size();
      if (absl::EndsWith(map_filename, ".xml")) {
        if (!apollo::hdmap::adapter::OpendriveAdapter::LoadData(map_filename,
                                                                &map_proto)) {
          return false;
        }
      } else if (!apollo::cyber::common::GetProtoFromFile(map_filename,
                                                          &map_proto)) {
        return false;
      }
      for (const auto &signal : map_proto.signal()) {
        const auto *hdmap = HDMapUtil::BaseMapPtr();
        if (!hdmap) {
          AERROR << "Invalid HD Map.";
          return false;
        }
        map_traffic_lights.push_back(hdmap->GetSignalById(signal.id()));
      }
    }
    *traffic_lights = map_traffic_lights;
    return true;
  }

  bool GetTrafficLightsWithinDistance(
      std::vector<SignalInfoConstPtr> *traffic_lights) {
    CHECK_NOTNULL(traffic_lights);
    if (!has_localization_) {
      AERROR << "No localization received";
      return false;
    }
    const auto *hdmap = HDMapUtil::BaseMapPtr();
    if (!hdmap) {
      AERROR << "Invalid HD Map.";
      return false;
    }
    auto position = localization_.pose().position();
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

  bool CreateTrafficLightDetection(
      const std::vector<SignalInfoConstPtr> &signals, TrafficLight::Color color,
      TrafficLightDetection *detection) {
    CHECK_NOTNULL(detection);
    for (const auto &iter : signals) {
      auto *light = detection->add_traffic_light();
      light->set_color(color);
      light->set_confidence(1.0);
      light->set_tracking_time(1.0);
      light->set_id(iter->id().id());
    }
    apollo::common::util::FillHeader("manual_traffic_light", detection);
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
          is_green_ = !is_green_;
          updated_ = true;
        }
        break;
    }
    if (is_green_) {
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

  void OnLocalization(
      const std::shared_ptr<LocalizationEstimate> &localization) {
    localization_ = *localization;
    has_localization_ = true;
  }

 private:
  bool is_green_ = false;
  bool updated_ = true;
  bool has_localization_ = false;
  LocalizationEstimate localization_;
  std::unordered_set<std::string> prev_traffic_lights_;
  std::shared_ptr<apollo::cyber::Writer<TrafficLightDetection>>
      traffic_light_detection_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<LocalizationEstimate>>
      localization_reader_ = nullptr;
};

CYBER_REGISTER_COMPONENT(ManualTrafficLight);
