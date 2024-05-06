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
#include "modules/dreamview_plus/backend/channels_updater/channels_updater.h"

#include <memory>
#include <string>
#include <vector>

// #include "google/protobuf/util/json_util.h"

#include "modules/dreamview_plus/proto/data_handler.pb.h"

#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"

namespace apollo {

namespace dreamview {

using apollo::common::util::ContainsKey;
using apollo::common::util::JsonUtil;

ChannelsUpdater::ChannelsUpdater(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("channels_info_updater")),
      websocket_(websocket) {}

void ChannelsUpdater::PublishMessage(const std::string &channel_name) {
  if (channel_updaters_.count(channel_name) == 0) {
    AERROR << "Failed to publish channel info for channel has not been "
              "registered!";
    return;
  }
  if (GetThrottleFlag()) {
    // Can continue to send data stream
    return;
  }
  // lock to avoid excessive stream data
  ControlThrottle(true);
  std::string channel_data_str =
      channel_updaters_[channel_name].channel_data_str;
  StreamData stream_data;
  std::string stream_data_string;
  stream_data.set_action("stream");
  stream_data.set_data_name("cyber");
  std::vector<uint8_t> byte_data(channel_data_str.begin(),
                                 channel_data_str.end());
  stream_data.set_data(&(byte_data[0]), byte_data.size());
  stream_data.set_type("cyber");
  stream_data.set_channel_name(channel_name);
  stream_data.SerializeToString(&stream_data_string);
  websocket_->BroadcastBinaryData(stream_data_string);
}

void ChannelsUpdater::ControlThrottle(bool add_lock) {
  {
    boost::unique_lock<boost::shared_mutex> wlock(throttle_mutex_);
    throttle = add_lock;
  }
}

bool ChannelsUpdater::GetThrottleFlag() {
  boost::shared_lock<boost::shared_mutex> rlock(throttle_mutex_);
  return throttle;
}

void ChannelsUpdater::StartStream(const double &time_interval_ms,
                                  const std::string &channel_name,
                                  nlohmann::json *subscribe_param) {
  if (!timer_initialized_) {
    timer_.reset(new cyber::Timer(
        time_interval_ms, [this]() { this->ControlThrottle(false); }, false));
    timer_->Start();
  }
  SubscribeChannel(channel_name);
}

bool ChannelsUpdater::SubscribeChannel(const std::string &channel_name) {
  if (channel_name.empty() || channel_updaters_.count(channel_name) == 1) {
    AERROR << "Invalid channel name or channel name has already "
              "registered,avoid dumplicate register!";
    return false;
  }
  struct ChannelUpdater channel_updater;
  channel_updaters_[channel_name] = channel_updater;
  auto cb = [this, channel_name](const std::shared_ptr<const RawMessage> &msg) {
    std::string str;
    msg->SerializeToString(&str);
    channel_updaters_[channel_name].channel_data_str = str;
    PublishMessage(channel_name);
  };
  auto reader = node_->CreateReader<RawMessage>(channel_name, cb);
  if (!reader) {
    channel_updaters_.erase(channel_name);
    return false;
  }
  channel_updaters_[channel_name].reader = reader;
  return true;
}

void ChannelsUpdater::StopStream(const std::string &channel_name) {
  if (channel_name.empty()) {
    AERROR << "Unsubscribe channelsInfo must bring channel param.";
    return;
  }
  if (channel_updaters_.count(channel_name) == 0) {
    AERROR << "This channel is unsubscribed,no need to unsubscribe.";
    return;
  }
  node_->DeleteReader(channel_name);
  channel_updaters_.erase(channel_name);
  return;
}
}  // namespace dreamview
}  // namespace apollo
