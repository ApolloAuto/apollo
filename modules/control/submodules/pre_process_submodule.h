/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/util/util.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_common_conf.pb.h"
// #include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace control {
class PreProcessSubmodule : public apollo::cyber::TimerComponent {
 public:
  /**
   * @brief Construct a new MPCControllerSubmodule object
   *
   */
  PreProcessSubmodule();
  /**
   * @brief Destructor
   */
  ~PreProcessSubmodule();

  /**
   * @brief Get name of the node
   * @return Name of the node
   */
  std::string Name() const;

  /**
   * @brief Initialize the node
   * @return If initialized
   */
  bool Init() override;

  /**
   * @brief generate control command
   *
   * @return true control command is successfully generated
   * @return false fail to generate control command
   */
  bool Proc() override;

  struct LocalView {
    canbus::Chassis chassis;
    planning::ADCTrajectory trajectory;
    localization::LocalizationEstimate localization;
  };

 private:
  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);

 private:
  double init_time_ = 0.0;

  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;
  common::Header latest_replan_trajectory_header_;

  std::mutex mutex_;

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::control::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> trajectory_reader_;
  std::shared_ptr<cyber::Writer<apollo::control::ControlCommand>>
      control_command_writer_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  // TODO(SHU): separate conf
  ControlCommonConf control_common_conf_;

  LocalView local_view_;
};

CYBER_REGISTER_COMPONENT(PreProcessSubmodule);
}  // namespace control
}  // namespace apollo
