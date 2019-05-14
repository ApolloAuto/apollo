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

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/util/util.h"
#include "modules/control/controller/controller_agent.h"

namespace apollo {
namespace bridge {

class BridgeControlComponent final : public apollo::cyber::TimerComponent {
 public:
  BridgeControlComponent();
  bool Init() override;
  bool Proc() override;

 private:
  void OnPad(const std::shared_ptr<apollo::control::PadMessage> &pad);

  template<typename T>
  bool CreateReader(std::shared_ptr<Reader<T>> reader, const std::string &topic,
    uint32_t queue_size) {
    cyber::ReaderConfig reader_config;
    reader_config.channel_name = topic;
    reader_config.pending_queue_size = queue_size;

    reader =
      node_->CreateReader<T>(reader_config, nullptr);
    CHECK(reader != nullptr);
    if (!reader) {
      return false;
    }
    return true;
  }

 private:
  double init_time_ = 0.0;

  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  control::PadMessage pad_msg_;

  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  std::mutex mutex_;

  std::shared_ptr<Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<Reader<apollo::control::PadMessage>> pad_msg_reader_;
  std::shared_ptr<Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<Reader<apollo::planning::ADCTrajectory>> trajectory_reader_;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

}  // namespace bridge
}  // namespace apollo
