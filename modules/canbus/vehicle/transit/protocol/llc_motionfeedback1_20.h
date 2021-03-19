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
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transit {

class Llcmotionfeedback120 : public ::apollo::drivers::canbus::ProtocolData<
                                 ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Llcmotionfeedback120();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;
  FRIEND_TEST(Motionfeedback1_20_test, General);

 private:
  // config detail: {'description': 'Current gear', 'enum': {0:
  // 'LLC_FBK_GEAR_P_PARK', 1: 'LLC_FBK_GEAR_D_DRIVE', 2:
  // 'LLC_FBK_GEAR_N_NEUTRAL', 3: 'LLC_FBK_GEAR_R_REVERSE'}, 'precision': 1.0,
  // 'len': 3, 'name': 'LLC_FBK_Gear', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 50, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Llc_motionfeedback1_20::Llc_fbk_gearType llc_fbk_gear(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Parking brake applied', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_ParkingBrake',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 53, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_parkingbrake(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': 'Throttle position feedback', 'offset': 0.0,
  // 'precision': 0.1, 'len': 10, 'name': 'LLC_FBK_ThrottlePosition',
  // 'is_signed_var': False, 'physical_range': '[0|102.3]', 'bit': 38, 'type':
  // 'double', 'order': 'intel', 'physical_unit': '%'}
  double llc_fbk_throttleposition(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Rear brake pressure feedback', 'offset':
  // 0.0, 'precision': 0.0556, 'len': 11, 'name': 'LLC_FBK_BrakePercentRear',
  // 'is_signed_var': False, 'physical_range': '[0|113.8132]', 'bit': 27,
  // 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
  double llc_fbk_brakepercentrear(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Front brake pressure feedback', 'offset':
  // 0.0, 'precision': 0.0556, 'len': 11, 'name': 'LLC_FBK_BrakePercentFront',
  // 'is_signed_var': False, 'physical_range': '[0|113.8132]', 'bit': 16,
  // 'type': 'double', 'order': 'intel', 'physical_unit': '%'}
  double llc_fbk_brakepercentfront(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'description': 'Current steering control mode', 'enum': {0:
  // 'LLC_FBK_STEERINGCONTROLMODE_NONE', 1: 'LLC_FBK_STEERINGCONTROLMODE_ANGLE',
  // 2: 'LLC_FBK_STEERINGCONTROLMODE_RESERVED_CURVATURE', 3:
  // 'LLC_FBK_STEERINGCONTROLMODE_RESERVED'}, 'precision': 1.0, 'len': 2,
  // 'name': 'LLC_FBK_SteeringControlMode', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Llc_motionfeedback1_20::Llc_fbk_steeringcontrolmodeType
  llc_fbk_steeringcontrolmode(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': 'Motion feedback 1 heartbeat counter',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'LLC_MotionFeedback1_Counter', 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_motionfeedback1_counter(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Motion feedback 1 checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'LLC_MotionFeedback1_Checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_motionfeedback1_checksum(const std::uint8_t* bytes,
                                   const int32_t length) const;

  // config detail: {'description': 'Autonomy command aligned with vehicle state
  // according to calibration limits', 'offset': 0.0, 'precision': 1.0, 'len':
  // 1, 'name': 'LLC_FBK_CommandAligned', 'is_signed_var': False,
  // 'physical_range': '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'intel',
  // 'physical_unit': 'T/F'}
  bool llc_fbk_commandaligned(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'description': 'Estop is pressed', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_EstopPressed',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_estoppressed(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': 'Indicates that ADC is requesting autonomy
  // mode', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'LLC_FBK_AdcRequestAutonomy', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit':
  // 'T/F'}
  bool llc_fbk_adcrequestautonomy(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Indicates that LLC is ready to allow
  // autonomy mode', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'LLC_FBK_allowAutonomy', 'is_signed_var': False, 'physical_range': '[0|1]',
  // 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_allowautonomy(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'description': 'Report current longitudinal control mode',
  // 'enum': {0: 'LLC_FBK_LONGITUDINALCONTROLMODE_NONE', 1:
  // 'LLC_FBK_LONGITUDINALCONTROLMODE_RESERVED_VELOCITY_AND_ACCELERATION', 2:
  // 'LLC_FBK_LONGITUDINALCONTROLMODE_RESERVED_FORCE', 3:
  // 'LLC_FBK_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE'}, 'precision': 1.0,
  // 'len': 2, 'name': 'LLC_FBK_LongitudinalControlMode', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Llc_motionfeedback1_20::Llc_fbk_longitudinalcontrolmodeType
  llc_fbk_longitudinalcontrolmode(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Current Autonomy State', 'enum': {0:
  // 'LLC_FBK_STATE_RESERVED0', 1: 'LLC_FBK_STATE_AUTONOMY_NOT_ALLOWED', 2:
  // 'LLC_FBK_STATE_AUTONOMY_ALLOWED', 3: 'LLC_FBK_STATE_AUTONOMY_REQUESTED', 4:
  // 'LLC_FBK_STATE_AUTONOMY', 5: 'LLC_FBK_STATE_RESERVED1', 6:
  // 'LLC_FBK_STATE_RESERVED2', 7: 'LLC_FBK_STATE_RESERVED3', 8:
  // 'LLC_FBK_STATE_RESERVED4', 9: 'LLC_FBK_STATE_RESERVED5', 10:
  // 'LLC_FBK_STATE_RESERVED6', 11: 'LLC_FBK_STATE_RESERVED7', 12:
  // 'LLC_FBK_STATE_RESERVED8', 13: 'LLC_FBK_STATE_DISENGAGE_REQUESTED', 14:
  // 'LLC_FBK_STATE_DISENGAGED', 15: 'LLC_FBK_STATE_FAULT'}, 'precision': 1.0,
  // 'len': 4, 'name': 'LLC_FBK_State', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|15]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Llc_motionfeedback1_20::Llc_fbk_stateType llc_fbk_state(
      const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
