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

/**
 * @file
 */

#pragma once
#include <memory>
#include <string>
#include <unordered_map>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_base.h"
/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

using apollo::cyber::message::RawMessage;

struct ChannelUpdater {
  std::shared_ptr<apollo::cyber::Reader<RawMessage>> reader;
  std::string channel_data_str;
  ChannelUpdater() : reader(nullptr), channel_data_str("") {}
};

/**
 * @class ChannelsUpdater
 * @brief A wrapper around WebSocketHandler to keep pushing specified channels info to
 * frontend via websocket.
 */
class ChannelsUpdater : public UpdaterBase {
 public:
  using ReaderMap = std::unordered_map<std::string, ChannelUpdater>;
  /**
   * @brief Constructor with the websocket handler.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   */
  explicit ChannelsUpdater(WebSocketHandler *websocket);
  ~ChannelsUpdater() override {
    if (timer_initialized_) {
      timer_->Stop();
    }
  }
  void StartStream(const double &time_interval_ms,
                   const std::string &channel_name = "",
                   nlohmann::json *subscribe_param = nullptr) override;
  void StopStream(const std::string& channel_name = "") override;
  void PublishMessage(const std::string& channel_name = "") override;

 private:
  bool SubscribeChannel(const std::string &channel_name);
  void ControlThrottle(bool add_lock);
  bool GetThrottleFlag();
  std::unique_ptr<cyber::Node> node_;
  WebSocketHandler *websocket_ = nullptr;
  ReaderMap channel_updaters_;
  // Channel data stream string by channels.
  bool timer_initialized_ = false;
  // Add channel timer to control channel data stream frequency.
  std::unique_ptr<cyber::Timer> timer_;
  // Channel data stream throttle flag
  bool throttle = false;
  mutable boost::shared_mutex throttle_mutex_;
};

}  // namespace dreamview
}  // namespace apollo
