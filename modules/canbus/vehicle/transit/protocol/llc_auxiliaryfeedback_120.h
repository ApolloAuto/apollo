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

class Llcauxiliaryfeedback120 : public ::apollo::drivers::canbus::ProtocolData<
                                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Llcauxiliaryfeedback120();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

  FRIEND_TEST(llc_auxiliaryfeedback_120Test, General);

 private:
  // config detail: {'description': 'Inverter enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_Inverter', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_inverter(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 8 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch8', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 15, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch8(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 7 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch7', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 14, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch7(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 6 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch6', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 13, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch6(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 5 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch5', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 12, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch5(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 4 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch4', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 11, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch4(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 3 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch3', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 10, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 2 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch2', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 9, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'PDU Channel 1 enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_PDU_Ch1', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 8, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_pdu_ch1(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Hazard lights enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_HazardLights',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 28, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_hazardlights(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': 'Autonomy indicator green LED on', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_LedGreenOn',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 17, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_ledgreenon(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'description': 'Horn enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_Horn', 'is_signed_var': False,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'intel',
  // 'physical_unit': 'T/F'}
  bool llc_fbk_horn(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Buzzer enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_BuzzerOn', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 19, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_buzzeron(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Current turn signal status', 'enum': {0:
  // 'LLC_FBK_TURNSIGNAL_NONE', 1: 'LLC_FBK_TURNSIGNAL_LEFT', 2:
  // 'LLC_FBK_TURNSIGNAL_RIGHT', 3: 'LLC_FBK_TURNSIGNAL_RESERVE'},
  // 'precision': 1.0, 'len': 2, 'name': 'LLC_FBK_TurnSignal', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Llc_auxiliaryfeedback_120::Llc_fbk_turnsignalType llc_fbk_turnsignal(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Low beam enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_LowBeam', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_lowbeam(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'High beam enabled', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_HighBeam', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_highbeam(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Autonomy indicator red LED on', 'offset':
  // 0.0, 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_LedRedOn',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 18, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_ledredon(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Autonomy button is pressed', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'LLC_FBK_AutonomyButtonPressed',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 16, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  bool llc_fbk_autonomybuttonpressed(const std::uint8_t* bytes,
                                     const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
