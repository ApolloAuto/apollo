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

#include "modules/canbus/vehicle/transit/protocol/llc_motionfeedback1_20.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcmotionfeedback120::Llcmotionfeedback120() {}
const int32_t Llcmotionfeedback120::ID = 0x20;

void Llcmotionfeedback120::Parse(const std::uint8_t* bytes, int32_t length,
                                 ChassisDetail* chassis) const {
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_gear(llc_fbk_gear(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_parkingbrake(llc_fbk_parkingbrake(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_throttleposition(llc_fbk_throttleposition(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_brakepercentrear(llc_fbk_brakepercentrear(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_brakepercentfront(llc_fbk_brakepercentfront(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_steeringcontrolmode(
          llc_fbk_steeringcontrolmode(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_motionfeedback1_counter(
          llc_motionfeedback1_counter(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_motionfeedback1_checksum(
          llc_motionfeedback1_checksum(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_commandaligned(llc_fbk_commandaligned(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_estoppressed(llc_fbk_estoppressed(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_adcrequestautonomy(
          llc_fbk_adcrequestautonomy(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_allowautonomy(llc_fbk_allowautonomy(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_longitudinalcontrolmode(
          llc_fbk_longitudinalcontrolmode(bytes, length));
  chassis->mutable_transit()
      ->mutable_llc_motionfeedback1_20()
      ->set_llc_fbk_state(llc_fbk_state(bytes, length));
}

// config detail: {'description': 'Current gear', 'enum': {0:
// 'LLC_FBK_GEAR_P_PARK', 1: 'LLC_FBK_GEAR_D_DRIVE', 2:
// 'LLC_FBK_GEAR_N_NEUTRAL', 3: 'LLC_FBK_GEAR_R_REVERSE'}, 'precision': 1.0,
// 'len': 3, 'name': 'llc_fbk_gear', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 50, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Llc_motionfeedback1_20::Llc_fbk_gearType Llcmotionfeedback120::llc_fbk_gear(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 3);

  Llc_motionfeedback1_20::Llc_fbk_gearType ret =
      static_cast<Llc_motionfeedback1_20::Llc_fbk_gearType>(x);
  return ret;
}

// config detail: {'description': 'Parking brake applied', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_parkingbrake', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 53, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcmotionfeedback120::llc_fbk_parkingbrake(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Throttle position feedback', 'offset': 0.0,
// 'precision': 0.1, 'len': 10, 'name': 'llc_fbk_throttleposition',
// 'is_signed_var': False, 'physical_range': '[0|102.3]', 'bit': 38, 'type':
// 'double', 'order': 'intel', 'physical_unit': '%'}
double Llcmotionfeedback120::llc_fbk_throttleposition(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'description': 'Rear brake pressure feedback', 'offset': 0.0,
// 'precision': 0.0556, 'len': 11, 'name': 'llc_fbk_brakepercentrear',
// 'is_signed_var': False, 'physical_range': '[0|113.8132]', 'bit': 27, 'type':
// 'double', 'order': 'intel', 'physical_unit': '%'}
double Llcmotionfeedback120::llc_fbk_brakepercentrear(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.055600;
  return ret;
}

// config detail: {'description': 'Front brake pressure feedback', 'offset':
// 0.0, 'precision': 0.0556, 'len': 11, 'name': 'llc_fbk_brakepercentfront',
// 'is_signed_var': False, 'physical_range': '[0|113.8132]', 'bit': 16, 'type':
// 'double', 'order': 'intel', 'physical_unit': '%'}
double Llcmotionfeedback120::llc_fbk_brakepercentfront(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.055600;
  return ret;
}

// config detail: {'description': 'Current steering control mode', 'enum': {0:
// 'LLC_FBK_STEERINGCONTROLMODE_NONE', 1: 'LLC_FBK_STEERINGCONTROLMODE_ANGLE',
// 2: 'LLC_FBK_STEERINGCONTROLMODE_RESERVED_CURVATURE', 3:
// 'LLC_FBK_STEERINGCONTROLMODE_RESERVED'}, 'precision': 1.0, 'len': 2, 'name':
// 'llc_fbk_steeringcontrolmode', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Llc_motionfeedback1_20::Llc_fbk_steeringcontrolmodeType
Llcmotionfeedback120::llc_fbk_steeringcontrolmode(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Llc_motionfeedback1_20::Llc_fbk_steeringcontrolmodeType ret =
      static_cast<Llc_motionfeedback1_20::Llc_fbk_steeringcontrolmodeType>(x);
  return ret;
}

// config detail: {'description': 'Motion feedback 1 heartbeat counter',
// 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
// 'llc_motionfeedback1_counter', 'is_signed_var': False, 'physical_range':
// '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Llcmotionfeedback120::llc_motionfeedback1_counter(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Motion feedback 1 checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'llc_motionfeedback1_checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Llcmotionfeedback120::llc_motionfeedback1_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'description': 'Autonomy command aligned with vehicle state
// according to calibration limits', 'offset': 0.0, 'precision': 1.0, 'len': 1,
// 'name': 'llc_fbk_commandaligned', 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
bool Llcmotionfeedback120::llc_fbk_commandaligned(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Estop is pressed', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_estoppressed', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 10, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcmotionfeedback120::llc_fbk_estoppressed(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Indicates that ADC is requesting autonomy
// mode', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
// 'llc_fbk_adcrequestautonomy', 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
bool Llcmotionfeedback120::llc_fbk_adcrequestautonomy(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Indicates that LLC is ready to allow autonomy
// mode', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
// 'llc_fbk_allowautonomy', 'is_signed_var': False, 'physical_range': '[0|1]',
// 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
bool Llcmotionfeedback120::llc_fbk_allowautonomy(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Report current longitudinal control mode',
// 'enum': {0: 'LLC_FBK_LONGITUDINALCONTROLMODE_NONE', 1:
// 'LLC_FBK_LONGITUDINALCONTROLMODE_RESERVED_VELOCITY_AND_ACCELERATION', 2:
// 'LLC_FBK_LONGITUDINALCONTROLMODE_RESERVED_FORCE', 3:
// 'LLC_FBK_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE'}, 'precision': 1.0,
// 'len': 2, 'name': 'llc_fbk_longitudinalcontrolmode', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order':
// 'intel', 'physical_unit': ''}
Llc_motionfeedback1_20::Llc_fbk_longitudinalcontrolmodeType
Llcmotionfeedback120::llc_fbk_longitudinalcontrolmode(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Llc_motionfeedback1_20::Llc_fbk_longitudinalcontrolmodeType ret =
      static_cast<Llc_motionfeedback1_20::Llc_fbk_longitudinalcontrolmodeType>(
          x);
  return ret;
}

// config detail: {'description': 'Current Autonomy State', 'enum': {0:
// 'LLC_FBK_STATE_RESERVED0', 1: 'LLC_FBK_STATE_AUTONOMY_NOT_ALLOWED', 2:
// 'LLC_FBK_STATE_AUTONOMY_ALLOWED', 3: 'LLC_FBK_STATE_AUTONOMY_REQUESTED', 4:
// 'LLC_FBK_STATE_AUTONOMY', 5: 'LLC_FBK_STATE_RESERVED1', 6:
// 'LLC_FBK_STATE_RESERVED2', 7: 'LLC_FBK_STATE_RESERVED3', 8:
// 'LLC_FBK_STATE_RESERVED4', 9: 'LLC_FBK_STATE_RESERVED5', 10:
// 'LLC_FBK_STATE_RESERVED6', 11: 'LLC_FBK_STATE_RESERVED7', 12:
// 'LLC_FBK_STATE_RESERVED8', 13: 'LLC_FBK_STATE_DISENGAGE_REQUESTED', 14:
// 'LLC_FBK_STATE_DISENGAGED', 15: 'LLC_FBK_STATE_FAULT'}, 'precision': 1.0,
// 'len': 4, 'name': 'llc_fbk_state', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|15]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Llc_motionfeedback1_20::Llc_fbk_stateType Llcmotionfeedback120::llc_fbk_state(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  Llc_motionfeedback1_20::Llc_fbk_stateType ret =
      static_cast<Llc_motionfeedback1_20::Llc_fbk_stateType>(x);
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
