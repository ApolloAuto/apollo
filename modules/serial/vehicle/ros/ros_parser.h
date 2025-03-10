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

namespace apollo {
namespace serial {

class ROSParser {
 public:
  ROSParser() = default;
  virtual ~ROSParser() = default;

  static bool Encode(const ControlCommand& cmd, uint8_t* data, size_t length);

  static bool DecodeTwistFb(const uint8_t* data, size_t length,
                            Chassis* chassis);

  static bool DecodeMiscFb(const uint8_t* data, size_t length,
                           Chassis* chassis);
};

}  // namespace serial
}  // namespace apollo
