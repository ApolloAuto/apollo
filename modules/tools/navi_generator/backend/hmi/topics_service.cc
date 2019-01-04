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
#include "modules/localization/msf/common/util/frame_transform.h"
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

namespace {
static constexpr double kAngleThreshold = 0.1;
// kSinsRadToDeg = 180 / pi
constexpr double kSinsRadToDeg = 57.295779513;
constexpr std::size_t kLocalUTMZoneID = 49;
}  // namespace

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

// A callback function which informs the bag files has been processed.
void TopicsService::NotifyBagFilesProcessed(void *service) {
  CHECK_NOTNULL(service);
  TopicsService *topics_service = reinterpret_cast<TopicsService *>(service);
  if (topics_service) {
    topics_service->CorrectRoadDeviation();
  }
}

TopicsService::TopicsService(NaviGeneratorWebSocket *websocket)
    : websocket_(websocket), ready_to_push_(false) {
  trajectory_processor_.reset(new util::TrajectoryProcessor(
      TopicsService::UpdateGUI, TopicsService::NotifyBagFilesProcessed, this));
  navigation_editor_.reset(
      new util::NavigationEditor(TopicsService::UpdateGUI, this));
  trajectory_collector_.reset(new util::TrajectoryCollector());
  navigation_provider_.reset(new util::NavigationProvider());
}

void TopicsService::Update() {
  const apollo::localization::LocalizationEstimate localization =
      trajectory_collector_->GetLocalization();
  if (!localization.has_pose()) {
    return;
  }
  if (!ReadyToSend()) {
    return;
  }
  UpdateObserved(localization);
  world_.set_sequence_num(world_.sequence_num() + 1);
  world_.set_timestamp(apollo::common::time::AsInt64<millis>(Clock::Now()));
  trajectory_collector_->PublishCollector();
}

void TopicsService::UpdateObserved(
    const apollo::localization::LocalizationEstimate &localization) {
  // Update status
  Object *auto_driving_car = world_.mutable_auto_driving_car();
  const auto &pose = localization.pose();

  apollo::localization::msf::WGS84Corr wgs84;
  apollo::localization::msf::UtmXYToLatlon(
      pose.position().x(), pose.position().y(), kLocalUTMZoneID, false, &wgs84);
  wgs84.lat *= kSinsRadToDeg;
  wgs84.log *= kSinsRadToDeg;

  // Updates position with the input localization message.
  auto_driving_car->set_position_x(pose.position().x());
  auto_driving_car->set_position_y(pose.position().y());
  auto_driving_car->set_heading(pose.heading());

  auto_driving_car->set_latitude(wgs84.lat);
  auto_driving_car->set_longitude(wgs84.log);

  // Updates the timestamp with the timestamp inside the localization
  // message header. It is done on both the SimulationWorld object
  // itself and its auto_driving_car() field.
  auto_driving_car->set_timestamp_sec(localization.header().timestamp_sec());

  ready_to_push_.store(true);
}

bool TopicsService::SetCommonBagFileInfo(
    const apollo::navi_generator::util::CommonBagFileInfo &common_file_info) {
  trajectory_processor_->Reset();
  SetReadyToSend(false);
  return trajectory_processor_->SetCommonBagFileInfo(common_file_info);
}

bool TopicsService::ProcessBagFileSegment(
    const apollo::navi_generator::util::FileSegment &file_segment) {
  return trajectory_processor_->ProcessBagFileSegment(file_segment);
}

bool TopicsService::SaveFilesToDatabase() {
  SetReadyToSend(true);
  return trajectory_processor_->SaveFilesToDatabase();
}

bool TopicsService::InitCollector() {
  if (!trajectory_collector_->Init()) {
    return false;
  }
  SetReadyToSend(true);
  return true;
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
    multiplier = 1000.0;
  } else {
    AWARN << "Collection type error.";
    return false;
  }
  std::vector<std::string> topics = {"/apollo/sensor/gnss/best_pose",
                                     "/apollo/localization/pose",
                                     "/apollo/navi_generator/collector"};
  options->topics = topics;
  if (is_duration) {
    options->collector_type = util::CollectorType::TIME;
    options->max_duration = ros::Duration(multiplier * duration);
  } else {
    options->collector_type = util::CollectorType::MILEAGE;
    options->max_mileage = multiplier * duration;
  }
  options->max_speed_limit = static_cast<double>(max_speed_limit);
  options->min_speed_limit = static_cast<double>(min_speed_limit);

  return true;
}

Json TopicsService::GetRoutePathAsJson(const Json &map_data) {
  Json response;
  navigation_provider_->GetRoutePathAsJson(map_data, false, &response);
  return response;
}

Json TopicsService::GetNavigationPathAsJson(const Json &map_data) {
  Json response;
  navigation_provider_->GetRoutePathAsJson(map_data, true, &response);
  return response;
}

Json TopicsService::GetUpdateAsJson() const {
  std::string sim_world_json_string;
  MessageToJsonString(world_, &sim_world_json_string);

  Json update;
  update["type"] = "SimWorldUpdate";
  update["data"] = sim_world_json_string;
  return update;
}

Json TopicsService::GetCommandResponseAsJson(const std::string &type,
                                             const std::string &module,
                                             const std::string &command,
                                             const int success) const {
  Json response;
  response["type"] = type;
  response["result"]["name"] = module;
  response["result"]["success"] = std::to_string(success);
  std::string msg;
  if (success == 0) {
    msg = module + " has been " + command + "successfully.";
  } else {
    msg = module + " has been " + command + "failed.";
  }
  response["result"]["msg"] = msg;
  return response;
}

bool TopicsService::CorrectRoadDeviation() {
  if (!road_deviation_correction_enabled_) {
    return true;
  }
  SetReadyToSend(false);
  road_deviation_correction_enabled_ = false;
  const std::map<std::uint16_t, apollo::navi_generator::util::FileInfo>
      *processed_file_info = nullptr;
  trajectory_processor_->GetProcessedFilesInfo(&processed_file_info);
  if (processed_file_info == nullptr) {
    return false;
  }
  if (!navigation_editor_->CorrectDeviation(*processed_file_info)) {
    return false;
  }
  return true;
}

bool TopicsService::SaveRoadCorrection() {
  SetReadyToSend(true);
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
