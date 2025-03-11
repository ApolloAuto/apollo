// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#pragma once

#include <memory>
#include <string>
#include <utility>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/serial/proto/serial_conf.pb.h"

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/serial/base_control.h"

namespace apollo {
namespace serial {

class SerialComponent final : public apollo::cyber::TimerComponent {
 public:
  using ControlCommand = apollo::control::ControlCommand;
  using Chassis = apollo::canbus::Chassis;

  SerialComponent();
  ~SerialComponent();
  /**
   * @brief obtain module name
   * @return module name
   */
  std::string Name() const;

 private:
  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init() override;

  /**
   * @brief module on_time function
   */
  bool Proc() override;

  /**
   * @brief module cleanup function
   */
  void Clear() override;

  void OnControlCommand(const apollo::control::ControlCommand &control_command);

  SerialConf serial_conf_;

  std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>>
      control_command_reader_;
  std::shared_ptr<cyber::Writer<Chassis>> chassis_writer_;

  std::unique_ptr<BaseControl> control_;
};

CYBER_REGISTER_COMPONENT(SerialComponent)

}  // namespace serial
}  // namespace apollo
