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

#include "modules/canbus_vehicle/transit/proto/transit.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

#include "gtest/gtest_prod.h"

namespace apollo {
namespace canbus {
namespace transit {

class Llcvehiclestatus25 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::canbus::Transit> {
 public:
  static const int32_t ID;
  Llcvehiclestatus25();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Transit* chassis) const override;

  FRIEND_TEST(llc_vehiclestatus_25Test, 12voltage);

 private:
  // config detail: {'description': 'Vehicle 12V voltage feedback', 'offset':
  // 0.0, 'precision': 0.1, 'len': 8, 'name': 'LLC_FBK_12Voltage',
  // 'is_signed_var': False, 'physical_range': '[0|25.5]', 'bit': 0, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'Volt'}
  double llc_fbk_12voltage(const std::uint8_t* bytes,
                           const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
