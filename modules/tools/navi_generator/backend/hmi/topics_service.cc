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

#include "modules/tools/navi_generator/backend/hmi/topics_service.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <unordered_set>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"
#include "modules/tools/navi_generator/backend/util/navigation_provider.h"

namespace apollo {
namespace navi_generator {

using apollo::common::time::Clock;
using apollo::common::time::millis;
using apollo::common::time::ToSecond;
using apollo::common::util::GetProtoFromFile;
using Json = nlohmann::json;
using apollo::common::adapter::AdapterManager;
using apollo::localization::LocalizationEstimate;
using google::protobuf::util::MessageToJsonString;

// A callback function which updates the GUI.
void TopicsService::UpdateGUI(const std::string &msg, void *service) {
  CHECK_NOTNULL(service);
  // Update the frontend through a WebSocket broadcast message.
  TopicsService *topics_service = reinterpret_cast<TopicsService *>(service);
  NaviGeneratorWebSocket *websocket = topics_service->websocket_;
  if (websocket) {
    websocket->BroadcastData(msg);
  }
}

TopicsService::TopicsService(NaviGeneratorWebSocket *websocket)
    : websocket_(websocket) {
  trajectory_processor_.reset(
      new util::TrajectoryProcessor(TopicsService::UpdateGUI, this));
}

void TopicsService::Update() {
  if (to_clear_) {
    // Clears received data.
    AdapterManager::GetLocalization()->ClearData();
    to_clear_ = false;
  }
}

template <>
void TopicsService::UpdateObserved(const LocalizationEstimate &localization) {
  // TODO(***): Update status
}

bool TopicsService::SetCommonBagFileInfo(
    const apollo::navi_generator::util::CommonBagFileInfo &common_file_info) {
  trajectory_processor_->Reset();
  return trajectory_processor_->SetCommonBagFileInfo(common_file_info);
}

bool TopicsService::ProcessBagFileSegment(
    const apollo::navi_generator::util::FileSegment &file_segment) {
  return trajectory_processor_->ProcessBagFileSegment(file_segment);
}

bool TopicsService::SaveFilesToDatabase() {
  return trajectory_processor_->SaveFilesToDatabase();
}

bool TopicsService::StartCollector(const std::string &collection_type,
                                   const std::size_t min_speed_limit,
                                   const std::size_t max_speed_limit) {
  util::CollectorOptions options;
  if (!GetCollectorOptions(collection_type, min_speed_limit, max_speed_limit,
                           &options)) {
    return false;
  }
  if (!trajectory_collector_->Init(options)) {
    return false;
  }
  if (!trajectory_collector_->Start()) {
    return false;
  }
  return true;
}

bool TopicsService::StopCollector() {
  if (!trajectory_collector_->Stop()) {
    return false;
  }
  return true;
}

bool TopicsService::UpdateCollector(const std::string &collection_type,
                                    const std::size_t min_speed_limit,
                                    const std::size_t max_speed_limit) {
  util::CollectorOptions options;
  if (!GetCollectorOptions(collection_type, min_speed_limit, max_speed_limit,
                           &options)) {
    return false;
  }
  if (!trajectory_collector_->UpdateOptions(options)) {
    return false;
  }
  return true;
}

bool TopicsService::GetCollectorOptions(const std::string &collection_type,
                                        const std::size_t min_speed_limit,
                                        const std::size_t max_speed_limit,
                                        util::CollectorOptions *const options) {
  CHECK_NOTNULL(options);
  std::string unit;
  double duration = 5.0;
  double multiplier = 60.0;
  bool is_duration = true;
  std::istringstream iss(collection_type);
  if ((iss >> duration).fail()) {
    AWARN << "Collection type must be start number.";
    return false;
  }
  if ((!iss.eof() && ((iss >> unit).fail()))) {
    AWARN << "Collection type unit must be min or km.";
    return false;
  }
  if (unit == std::string("")) {
    multiplier = 60.0;
  } else if (unit == std::string("min")) {
    is_duration = true;
  } else if (unit == std::string("km")) {
    is_duration = false;
  } else {
    AWARN << "Collection type error.";
    return false;
  }
  std::vector<std::string> topics = {"/apollo/sensor/gnss/best_pose",
                                     "/apollo/localization/pose",
                                     "/apollo/navi_generator/collector"};
  options->topics = topics;
  if (is_duration) {
    options->max_duration = ros::Duration(multiplier * duration);
  } else {
    options->max_mileage = duration;
  }
  options->max_speed_limit = static_cast<double>(max_speed_limit);
  options->min_speed_limit = static_cast<double>(min_speed_limit);

  return true;
}

Json TopicsService::GetRoutePathAsJson(const Json &map_data) {
  Json response;
  std::unique_ptr<util::NavigationProvider> provider;
  provider->GetRoutePathAsJson(map_data, &response);
  return response;
}

bool TopicsService::CorrectRoadDeviation() {
  const std::map<std::uint16_t, apollo::navi_generator::util::FileInfo>
      *processed_file_info = nullptr;
  trajectory_processor_->GetProcessedFilesInfo(&processed_file_info);
  if (processed_file_info == nullptr) {
    return false;
  }
  if (navigation_editor_->CorrectDeviation(*processed_file_info)) {
    return false;
  }
  return true;
}
bool TopicsService::SaveRoadCorrection() {
  if (!navigation_editor_->SaveRoadCorrection(trajectory_processor_.get())) {
    return false;
  }
  return true;
}

bool TopicsService::ModifySpeedLimit(
    const apollo::localization::msf::WGS84Corr &start_point,
    const apollo::localization::msf::WGS84Corr &end_point,
    const std::uint8_t new_speed_min, const std::uint8_t new_speed_max) {
  if (!navigation_editor_->ModifySpeedLimit(start_point, end_point,
                                            new_speed_min, new_speed_max)) {
    return false;
  }
  return true;
}

bool TopicsService::SaveSpeedLimit() {
  if (!navigation_editor_->SaveSpeedLimit()) {
    return false;
  }
  return true;
}

}  // namespace navi_generator
}  // namespace apollo
