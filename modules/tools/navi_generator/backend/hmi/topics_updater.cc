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

#include "modules/tools/navi_generator/backend/hmi/topics_updater.h"

#include "google/protobuf/util/json_util.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/util.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace navi_generator {

using apollo::common::adapter::AdapterManager;
using apollo::common::util::ContainsKey;
using apollo::common::util::GetProtoFromASCIIFile;
using apollo::common::util::JsonUtil;
using Json = nlohmann::json;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;

namespace {
// Time interval, in milliseconds, between pushing SimulationWorld to
// frontend.
constexpr double kSimWorldTimeIntervalMs = 100;
}  // namespace

TopicsUpdater::TopicsUpdater(NaviGeneratorWebSocket *websocket)
    : websocket_(websocket), topicsService_(websocket) {
  RegisterMessageHandlers();
}

void TopicsUpdater::RegisterMessageHandlers() {
  websocket_->RegisterMessageHandler(
      "requestStartCollection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        std::string collection_type;
        std::size_t min_speed_limit;
        std::size_t max_speed_limit;
        if (!JsonUtil::GetStringFromJson(json, "collectionType",
                                         &collection_type)) {
          return;
        }
        if (!JsonUtil::GetNumberFromJson(json, "lowSpeedLimit",
                                         &min_speed_limit)) {
          return;
        }
        if (!JsonUtil::GetNumberFromJson(json, "highSpeedLimit",
                                         &max_speed_limit)) {
          return;
        }
        topicsService_.StartCollector(collection_type, min_speed_limit,
                                      max_speed_limit);
      });

  websocket_->RegisterMessageHandler(
      "requestUpdateCollectionCondition",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        std::string collection_type;
        std::size_t min_speed_limit;
        std::size_t max_speed_limit;
        if (!JsonUtil::GetStringFromJson(json, "collectionType",
                                         &collection_type)) {
          return;
        }
        if (!JsonUtil::GetNumberFromJson(json, "lowSpeedLimit",
                                         &min_speed_limit)) {
          return;
        }
        if (!JsonUtil::GetNumberFromJson(json, "highSpeedLimit",
                                         &max_speed_limit)) {
          return;
        }
        topicsService_.UpdateCollector(collection_type, min_speed_limit,
                                       max_speed_limit);
      });

  websocket_->RegisterMessageHandler(
      "requestFinishCollection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        topicsService_.StopCollector();
      });

  websocket_->RegisterMessageHandler(
      "requestProcessBagFiles",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        if (ContainsKey(json, "totalFileNum")) {
          std::size_t num;
          apollo::navi_generator::util::CommonBagFileInfo common_file_info;
          if (!JsonUtil::GetNumberFromJson(json, "totalFileNum", &num)) {
            return;
          }
          common_file_info.total_file_num = num;
          if (!JsonUtil::GetNumberFromJson(json, "leftLaneNum", &num)) {
            return;
          }
          common_file_info.left_lane_num = num;
          if (!JsonUtil::GetNumberFromJson(json, "rightLaneNum", &num)) {
            return;
          }
          common_file_info.right_lane_num = num;
          topicsService_.SetCommonBagFileInfo(common_file_info);
        } else if (ContainsKey(json, "curIndex")) {
          std::string str;
          std::size_t num;
          apollo::navi_generator::util::FileSegment file_segment;
          if (!JsonUtil::GetNumberFromJson(json, "curIndex", &num)) {
            return;
          }
          file_segment.cur_file_index = num;
          if (!JsonUtil::GetNumberFromJson(json, "curSegCount", &num)) {
            return;
          }
          file_segment.cur_file_seg_count = num;
          if (!JsonUtil::GetNumberFromJson(json, "curSegIdx", &num)) {
            return;
          }
          file_segment.cur_file_seg_index = num;
          if (!JsonUtil::GetStringFromJson(json, "curName", &str)) {
            return;
          }
          file_segment.cur_file_name = str;
          if (!JsonUtil::GetStringFromJson(json, "nextName", &str)) {
            return;
          }
          file_segment.next_file_name = str;
          if (!JsonUtil::GetStringFromJson(json, "data", &str)) {
            return;
          }
          file_segment.cur_file_seg_data = str;
          topicsService_.ProcessBagFileSegment(file_segment);
        }
      });
  websocket_->RegisterMessageHandler(
      "requestSaveBagFiles",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        topicsService_.SaveFilesToDatabase();
      });

  websocket_->RegisterMessageHandler(
      "requestRoute",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        Json response = topicsService_.GetRoutePathAsJson(json);
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "requestNavigation",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): return the navigation result
      });

  websocket_->RegisterMessageHandler(
      "requestModifySpeedLimit",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        apollo::localization::msf::WGS84Corr start_point;
        apollo::localization::msf::WGS84Corr end_point;
        std::uint8_t new_speed_min;
        std::uint8_t new_speed_max;

        if (!ContainsKey(json, "start") || !ContainsKey(json, "end")) {
          AERROR << "Failed to find start or end point.";
          return;
        }
        if (!ValidateCoordinate(json["start"])) {
          return;
        }
        if (!ValidateCoordinate(json["end"])) {
          return;
        }
        start_point.log = json["start"]["Lng"];
        start_point.lat = json["start"]["Lat"];
        end_point.log = json["start"]["Lng"];
        end_point.lat = json["start"]["Lat"];
        if (!JsonUtil::GetNumberFromJson(json, "speed_min", &new_speed_min)) {
          return;
        }
        if (!JsonUtil::GetNumberFromJson(json, "speed_max", &new_speed_max)) {
          return;
        }
        topicsService_.ModifySpeedLimit(start_point, end_point, new_speed_min,
                                        new_speed_max);
      });

  websocket_->RegisterMessageHandler(
      "requestSaveSpeedLimitCorrection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        topicsService_.SaveSpeedLimit();
      });

  websocket_->RegisterMessageHandler(
      "requestCorrectRoadDeviation",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        topicsService_.CorrectRoadDeviation();
      });

  websocket_->RegisterMessageHandler(
      "requestSaveRoadCorrection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        topicsService_.SaveRoadCorrection();
      });
}

bool TopicsUpdater::ValidateCoordinate(const nlohmann::json &json) {
  if (!ContainsKey(json, "x") || !ContainsKey(json, "y")) {
    AERROR << "Failed to find x or y coordinate.";
    return false;
  }
  if (json.find("x")->is_number() && json.find("y")->is_number()) {
    return true;
  }
  AERROR << "Both x and y coordinate should be a number.";
  return false;
}

void TopicsUpdater::Start() {
  // start ROS timer, one-shot = false, auto-start = true
  timer_ =
      AdapterManager::CreateTimer(ros::Duration(kSimWorldTimeIntervalMs / 1000),
                                  &TopicsUpdater::OnTimer, this);
}

void TopicsUpdater::OnTimer(const ros::TimerEvent &event) {
  topicsService_.Update();
}

}  // namespace navi_generator
}  // namespace apollo
