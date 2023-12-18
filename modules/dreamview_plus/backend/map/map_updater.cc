/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include<memory>
#include<string>
#include<vector>
#include "google/protobuf/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/json_util.h"
#include "modules/dreamview_plus/backend/map/map_updater.h"

#include "modules/common_msgs/localization_msgs/localization.pb.h"

namespace apollo {

namespace dreamview {

using google::protobuf::util::JsonStringToMessage;
using apollo::common::util::ContainsKey;
using apollo::common::util::JsonUtil;

MapUpdater::MapUpdater(WebSocketHandler *websocket,
                       const MapService *map_service)
    : node_(cyber::CreateNode("map_updater")),
      websocket_(websocket),
      map_service_(map_service) {
  localization_reader_ =
      node_->CreateReader<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic,
          [this](
              const std::shared_ptr<apollo::localization::LocalizationEstimate>
                  &msg) { UpdateAdcPosition(msg); });
}

void MapUpdater::UpdateAdcPosition(
    const std::shared_ptr<apollo::localization::LocalizationEstimate>
        &localization) {
  const auto &pose = localization->pose();
  adc_point_.set_x(pose.position().x() + map_service_->GetXOffset());
  adc_point_.set_y(pose.position().y() + map_service_->GetYOffset());
}

void MapUpdater::PublishMessage(const std::string& channel_name) {
  auto retrieved = map_service_->RetrieveMapElements(map_element_ids_);
  retrieved.SerializeToString(&retrieved_map_string_);
  StreamData stream_data;
  std::string stream_data_string;
  stream_data.set_action("stream");
  stream_data.set_data_name("map");
  std::vector<uint8_t> byte_data(retrieved_map_string_.begin(),
                                 retrieved_map_string_.end());
  stream_data.set_data(&(byte_data[0]), byte_data.size());
  stream_data.set_type("map");
  stream_data.SerializeToString(&stream_data_string);
  websocket_->BroadcastBinaryData(stream_data_string);
}

void MapUpdater::OnTimer(const std::string& channel_name) { PublishMessage(); }

void MapUpdater::StartStream(const double &time_interval_ms,
                             const std::string &channel_name,
                             nlohmann::json *subscribe_param) {
  if (subscribe_param == nullptr ||
      !ContainsKey(*subscribe_param, "mapElementIds")) {
    AERROR << "Subscribe map must bring elementIds param";
    return;
  }
  // json to message
  map_element_ids_.Clear();
  nlohmann::json element_ids_json;
  if (!JsonUtil::GetJsonByPath(*subscribe_param, {"mapElementIds"},
                               &element_ids_json)) {
    AERROR << "ElementIds param is wrong type.";
    return;
  }
  if (!JsonStringToMessage(element_ids_json.dump(), &map_element_ids_).ok()) {
    AERROR << "Failed to parse elementIds from json!";
    return;
  }
  if (time_interval_ms > 0) {
    timer_.reset(new cyber::Timer(
        time_interval_ms, [this]() { this->OnTimer(); }, false));
    timer_->Start();
  } else {
    this->OnTimer();
  }
}

void MapUpdater::StopStream(const std::string &channel_name) {
  if (timer_) {
    timer_->Stop();
  }
}
}  // namespace dreamview
}  // namespace apollo
