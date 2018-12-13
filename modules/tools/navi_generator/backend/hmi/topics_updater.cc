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
    : websocket_(websocket),
      topicsService_(websocket) {
  RegisterMessageHandlers();
}

void TopicsUpdater::RegisterMessageHandlers() {
  websocket_->RegisterMessageHandler(
      "requestStartCollection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): start collectting trajectories online
      });

  websocket_->RegisterMessageHandler(
      "requestUpdateCollectionCondition",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): update collectting trajectories condition
      });

  websocket_->RegisterMessageHandler(
      "requestFinishCollection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): stop collectting trajectories
      });

  websocket_->RegisterMessageHandler(
      "requestProcessBagFiles",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): process bag files
      });
  websocket_->RegisterMessageHandler(
      "requestSaveBagFiles",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): save bag files
      });

  websocket_->RegisterMessageHandler(
      "requestRoute",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): return the route result
      });

  websocket_->RegisterMessageHandler(
      "requestNavigation",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): return the navigation result
      });

  websocket_->RegisterMessageHandler(
      "requestModifySpeedLimit",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): modify speed limit between starting and end point.
      });

  websocket_->RegisterMessageHandler(
      "requestSaveSpeedLimitCorrection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): save the correction of speed limit between starting
        // and end point.
      });

  websocket_->RegisterMessageHandler(
      "requestCorrectRoadDeviation",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): correct road deviation with bag files.
      });

  websocket_->RegisterMessageHandler(
      "requestSaveRoadCorrection",
      [this](const Json &json, NaviGeneratorWebSocket::Connection *conn) {
        // TODO(*): save the correction of road deviation with bag files.
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
