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

#include "gtest/gtest_prod.h"
#include "modules/canbus_vehicle/transit/proto/transit.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transit {

class Llcmotionfeedback221 : public ::apollo::drivers::canbus::ProtocolData<
                                 ::apollo::canbus::Transit> {
 public:
  static const int32_t ID;
  Llcmotionfeedback221();
  void Parse(const std::uint8_t* bytes, int32_t length,
             Transit* chassis) const override;

  FRIEND_TEST(llc_motionfeedback2_21Test, motion_fdk);

 private:
  // config detail: {'name': 'LLC_FBK_VehicleSpeed', 'offset': 0.0, 'precision':
  // 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|655.35]',
  // 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s'}
  double llc_fbk_vehiclespeed(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': 'Motion feedback 2 heartbeat counter',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'LLC_MotionFeedback2_Counter', 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_motionfeedback2_counter(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Motion feedback 2 checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'LLC_MotionFeedback2_Checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_motionfeedback2_checksum(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'description': 'Steer wheel angle feedback from SbW motor
  // (? rate)', 'offset': 0.0, 'precision': 0.05, 'len': 16, 'name':
  // 'LLC_FBK_SteeringRate', 'is_signed_var': True, 'physical_range':
  // '[-1638.4|1638.3]', 'bit': 16, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'deg/s'}
  double llc_fbk_steeringrate(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': 'Steering angle feedback', 'offset': 0.0,
  // 'precision': 0.05, 'len': 16, 'name': 'LLC_FBK_SteeringAngle',
  // 'is_signed_var': True, 'physical_range': '[-1638.4|1638.35]', 'bit': 0,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
  double llc_fbk_steeringangle(const std::uint8_t* bytes,
                               const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
