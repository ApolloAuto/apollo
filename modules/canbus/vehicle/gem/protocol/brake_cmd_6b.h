/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

class Brakecmd6b : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Brakecmd6b();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'BRAKE_CMD', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 7,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  Brakecmd6b* set_brake_cmd(double brake_cmd);

 private:
  // config detail: {'name': 'BRAKE_CMD', 'offset': 0.0, 'precision': 0.001,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 7,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  void set_p_brake_cmd(uint8_t* data, double brake_cmd);

 private:
  double brake_cmd_;
};

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
