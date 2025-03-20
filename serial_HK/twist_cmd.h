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

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace serial {

void set_x_target_speed(uint8_t* data, double x_target_speed);

void set_y_target_speed(uint8_t* data, double y_target_speed);

void set_angular_velocity_z(uint8_t* data, double z_angular_velocity);

void set_checksum(uint8_t* data);

}  // namespace serial
}  // namespace apollo
