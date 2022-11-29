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

#include "modules/canbus_vehicle/transit/protocol/llc_auxiliaryfeedback_120.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

Llcauxiliaryfeedback120::Llcauxiliaryfeedback120() {}
const int32_t Llcauxiliaryfeedback120::ID = 0x120;

void Llcauxiliaryfeedback120::Parse(const std::uint8_t* bytes, int32_t length,
                                    Transit* chassis) const {
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_inverter(
      llc_fbk_inverter(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch8(
      llc_fbk_pdu_ch8(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch7(
      llc_fbk_pdu_ch7(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch6(
      llc_fbk_pdu_ch6(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch5(
      llc_fbk_pdu_ch5(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch4(
      llc_fbk_pdu_ch4(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch3(
      llc_fbk_pdu_ch3(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch2(
      llc_fbk_pdu_ch2(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_pdu_ch1(
      llc_fbk_pdu_ch1(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_hazardlights(
      llc_fbk_hazardlights(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_ledgreenon(
      llc_fbk_ledgreenon(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_horn(
      llc_fbk_horn(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_buzzeron(
      llc_fbk_buzzeron(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_turnsignal(
      llc_fbk_turnsignal(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_lowbeam(
      llc_fbk_lowbeam(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_highbeam(
      llc_fbk_highbeam(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()->set_llc_fbk_ledredon(
      llc_fbk_ledredon(bytes, length));
  chassis->mutable_llc_auxiliaryfeedback_120()
      ->set_llc_fbk_autonomybuttonpressed(
          llc_fbk_autonomybuttonpressed(bytes, length));
}

// config detail: {'description': 'Inverter enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_inverter', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'intel',
// 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_inverter(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 8 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch8', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 15, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch8(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 7 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch7', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 14, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch7(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 6 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch6', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 13, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch6(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 5 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch5', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 12, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch5(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 4 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch4', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 11, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch4(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 3 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch3', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 10, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch3(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 2 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch2', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'intel',
// 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch2(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'PDU Channel 1 enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_pdu_ch1', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'intel',
// 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_pdu_ch1(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Hazard lights enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_hazardlights', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 28, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_hazardlights(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Autonomy indicator green LED on', 'offset':
// 0.0, 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_ledgreenon',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 17, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_ledgreenon(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Horn enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_horn', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'intel',
// 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_horn(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Buzzer enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_buzzeron', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 19, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_buzzeron(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Current turn signal status', 'enum': {0:
// 'LLC_FBK_TURNSIGNAL_NONE', 1: 'LLC_FBK_TURNSIGNAL_LEFT', 2:
// 'LLC_FBK_TURNSIGNAL_RIGHT', 3: 'LLC_FBK_TURNSIGNAL_RESERVE'},
// 'precision': 1.0, 'len': 2, 'name': 'llc_fbk_turnsignal', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
Llc_auxiliaryfeedback_120::Llc_fbk_turnsignalType
Llcauxiliaryfeedback120::llc_fbk_turnsignal(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  Llc_auxiliaryfeedback_120::Llc_fbk_turnsignalType ret =
      static_cast<Llc_auxiliaryfeedback_120::Llc_fbk_turnsignalType>(x);
  return ret;
}

// config detail: {'description': 'Low beam enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_lowbeam', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_lowbeam(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'High beam enabled', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_highbeam', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_highbeam(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Autonomy indicator red LED on', 'offset':
// 0.0, 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_ledredon', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 18, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_ledredon(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': 'Autonomy button is pressed', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'llc_fbk_autonomybuttonpressed',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 16, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
bool Llcauxiliaryfeedback120::llc_fbk_autonomybuttonpressed(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace transit
}  // namespace canbus
}  // namespace apollo
