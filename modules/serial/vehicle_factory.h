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

#include "modules/serial/vehicle/ros/ros_control.h"

namespace apollo {
namespace serial {

class VehicleFactory {
 public:
  static std::unique_ptr<BaseControl> CreateControl(
      const SerialConf& serial_conf) {
    switch (serial_conf.vehicle_type()) {
      case VehicleType::ROS:
        return std::make_unique<ROSControl>(serial_conf);
      default:
        return nullptr;
    }
  }
};

}  // namespace serial
}  // namespace apollo
