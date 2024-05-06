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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/common/macros.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common_msgs/dreamview_msgs/hmi_config.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_mode.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_status.pb.h"
#include "modules/common_msgs/monitor_msgs/system_status.pb.h"

/**
 * @namespace apollo::monitor
 * @brief apollo::monitor
 */
namespace apollo {
namespace monitor {

// Centralized monitor config and status manager.
class MonitorManager {
 public:
  void Init(const std::shared_ptr<apollo::cyber::Node>& node);

  // Start and end a monitoring frame.
  bool StartFrame(const double current_time);
  void EndFrame();

  // Getters.
  const apollo::dreamview::HMIMode& GetHMIMode() const { return mode_config_; }
  bool IsInAutonomousMode() const { return in_autonomous_driving_; }
  SystemStatus* GetStatus() { return &status_; }
  apollo::common::monitor::MonitorLogBuffer& LogBuffer() { return log_buffer_; }

  // Cyber reader / writer creator.
  template <class T>
  std::shared_ptr<cyber::Reader<T>> CreateReader(const std::string& channel) {
    if (readers_.find(channel) == readers_.end()) {
      readers_.emplace(channel, node_->CreateReader<T>(channel));
    }
    return std::dynamic_pointer_cast<cyber::Reader<T>>(readers_[channel]);
  }

  template <class T>
  std::shared_ptr<cyber::Writer<T>> CreateWriter(const std::string& channel) {
    return node_->CreateWriter<T>(channel);
  }

 private:
  SystemStatus status_;

  // Input statuses.
  std::string current_mode_;
  const apollo::dreamview::HMIConfig hmi_config_;
  apollo::dreamview::HMIMode mode_config_;
  bool in_autonomous_driving_ = false;
  bool CheckAutonomousDriving(const double current_time);

  apollo::common::monitor::MonitorLogBuffer log_buffer_;
  std::shared_ptr<apollo::cyber::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<cyber::ReaderBase>> readers_;

  DECLARE_SINGLETON(MonitorManager)
};

}  // namespace monitor
}  // namespace apollo
