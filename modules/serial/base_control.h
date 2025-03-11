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

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/serial/proto/serial_conf.pb.h"

#include "modules/serial/common/serial_stream.h"

namespace apollo {
namespace serial {

class BaseControl {
 public:
  using ControlCommand = apollo::control::ControlCommand;
  using Chassis = apollo::canbus::Chassis;
  BaseControl(const SerialConf& serial_conf) : serial_conf_(serial_conf) {
    serial_stream_.reset(
        new SerialStream(serial_conf_.device_name().c_str(),
                         get_serial_baudrate(serial_conf_.baud_rate()),
                         serial_conf_.timeout_usec()));
  }

  virtual ~BaseControl() = default;

  virtual ::apollo::common::ErrorCode Start() = 0;

  virtual void Stop() = 0;

  virtual bool Send(const ControlCommand& control_command) = 0;

  virtual Chassis GetChassis() = 0;

 protected:
  SerialConf serial_conf_;

  std::unique_ptr<SerialStream> serial_stream_;
};

}  // namespace serial
}  // namespace apollo
