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
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/control/submodules/preprocessor_submodule.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class ControlComponent final : public apollo::cyber::TimerComponent {
  friend class ControlTestBase;

 public:
  ControlComponent();
  bool Init() override;

  bool Proc() override;

 private:
  // Upon receiving pad message
  void OnPad(const std::shared_ptr<PadMessage> &pad);

  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);

  void OnPlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory);

  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);

  // Upon receiving monitor message
  void OnMonitor(
      const apollo::common::monitor::MonitorMessage &monitor_message);

  common::Status ProduceControlCommand(ControlCommand *control_command);
  common::Status CheckInput(LocalView *local_view);
  common::Status CheckTimestamp(const LocalView &local_view);
  common::Status CheckPad();

 private:
  absl::Time init_time_;

  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;
  common::Header latest_replan_trajectory_header_;

  ControllerAgent controller_agent_;

  bool estop_ = false;
  std::string estop_reason_;
  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  ControlConf control_conf_;

  std::mutex mutex_;

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      trajectory_reader_;

  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
  // when using control submodules
  std::shared_ptr<cyber::Writer<LocalView>> local_view_writer_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  LocalView local_view_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
