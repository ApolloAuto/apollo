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

class Adcauxiliarycontrol110 : public ::apollo::drivers::canbus::ProtocolData<
                                   ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adcauxiliarycontrol110();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  FRIEND_TEST(Auxiliarycontrol_110_test, General);

  // config detail: {'description': 'Aux control heartbeat counter', 'offset':
  // 0.0, 'precision': 1.0, 'len': 2, 'name': 'ADC_AuxControl_Counter',
  // 'is_signed_var': False, 'physical_range': '[0|3]', 'bit': 54, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  Adcauxiliarycontrol110* set_adc_auxcontrol_counter(
      int adc_auxcontrol_counter);

  // config detail: {'description': 'Aux control checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'ADC_AuxControl_Checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  Adcauxiliarycontrol110* set_adc_auxcontrol_checksum(
      int adc_auxcontrol_checksum);

  // config detail: {'description': 'Control inverter override (default ON if
  // not overridden)', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'ADC_CMD_Inverter_ControlEnable', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit':
  // 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_inverter_controlenable(
      bool adc_cmd_inverter_controlenable);

  // config detail: {'description': 'Control inverter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_Inverter', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_inverter(bool adc_cmd_inverter);

  // config detail: {'description': '(Reserved) Control wiper', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_Wiper', 'is_signed_var':
  // False, 'physical_range': '[0|3]', 'bit': 4, 'type': 'int', 'order':
  // 'intel', 'physical_unit': ''}
  Adcauxiliarycontrol110* set_adc_cmd_wiper(int adc_cmd_wiper);

  // config detail: {'description': 'PDU Control Override (all channels default
  // ON if not overridden)', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'ADC_CMD_PDU_ControlEnable', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit':
  // 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_controlenable(
      bool adc_cmd_pdu_controlenable);

  // config detail: {'description': 'Control PDU Ch 8 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch8',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 15, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch8(bool adc_cmd_pdu_ch8);

  // config detail: {'description': 'Control PDU Ch 7 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch7',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 14, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch7(bool adc_cmd_pdu_ch7);

  // config detail: {'description': 'Control PDU Ch 6 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch6',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 13, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch6(bool adc_cmd_pdu_ch6);

  // config detail: {'description': 'Control PDU Ch 5 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch5',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 12, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch5(bool adc_cmd_pdu_ch5);

  // config detail: {'description': 'Control PDU Ch 4 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch4',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 11, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch4(bool adc_cmd_pdu_ch4);

  // config detail: {'description': 'Control PDU Ch 3 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch3',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch3(bool adc_cmd_pdu_ch3);

  // config detail: {'description': 'Control PDU Ch 2 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch2',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 9, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch2(bool adc_cmd_pdu_ch2);

  // config detail: {'description': 'Control PDU Ch 1 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch1',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 8, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_pdu_ch1(bool adc_cmd_pdu_ch1);

  // config detail: {'description': 'Control hazard lights', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_HazardLights',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 28, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_hazardlights(bool adc_cmd_hazardlights);

  // config detail: {'description': 'Control high beam', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_HighBeam', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_highbeam(bool adc_cmd_highbeam);

  // config detail: {'description': 'Control low beam', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_LowBeam', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_lowbeam(bool adc_cmd_lowbeam);

  // config detail: {'description': 'Control horn', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_Horn', 'is_signed_var': False,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'intel',
  // 'physical_unit': 'T/F'}
  Adcauxiliarycontrol110* set_adc_cmd_horn(bool adc_cmd_horn);

  // config detail: {'description': 'Requested turn signals', 'enum': {0:
  // 'ADC_CMD_TURNSIGNAL_NONE', 1: 'ADC_CMD_TURNSIGNAL_LEFT', 2:
  // 'ADC_CMD_TURNSIGNAL_RIGHT', 3: 'ADC_CMD_TURNSIGNAL_RESERVE'},
  // 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_TurnSignal', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Adcauxiliarycontrol110* set_adc_cmd_turnsignal(
      Adc_auxiliarycontrol_110::Adc_cmd_turnsignalType adc_cmd_turnsignal);

 private:
  // config detail: {'description': 'Aux control heartbeat counter', 'offset':
  // 0.0, 'precision': 1.0, 'len': 2, 'name': 'ADC_AuxControl_Counter',
  // 'is_signed_var': False, 'physical_range': '[0|3]', 'bit': 54, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_auxcontrol_counter(uint8_t* data, int adc_auxcontrol_counter);

  // config detail: {'description': 'Aux control checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'ADC_AuxControl_Checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_auxcontrol_checksum(uint8_t* data,
                                     int adc_auxcontrol_checksum);

  // config detail: {'description': 'Control inverter override (default ON if
  // not overridden)', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'ADC_CMD_Inverter_ControlEnable', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit':
  // 'T/F'}
  void set_p_adc_cmd_inverter_controlenable(
      uint8_t* data, bool adc_cmd_inverter_controlenable);

  // config detail: {'description': 'Control inverter', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_Inverter', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_inverter(uint8_t* data, bool adc_cmd_inverter);

  // config detail: {'description': '(Reserved) Control wiper', 'offset': 0.0,
  // 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_Wiper', 'is_signed_var':
  // False, 'physical_range': '[0|3]', 'bit': 4, 'type': 'int', 'order':
  // 'intel', 'physical_unit': ''}
  void set_p_adc_cmd_wiper(uint8_t* data, int adc_cmd_wiper);

  // config detail: {'description': 'PDU Control Override (all channels default
  // ON if not overridden)', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
  // 'ADC_CMD_PDU_ControlEnable', 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit':
  // 'T/F'}
  void set_p_adc_cmd_pdu_controlenable(uint8_t* data,
                                       bool adc_cmd_pdu_controlenable);

  // config detail: {'description': 'Control PDU Ch 8 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch8',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 15, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch8(uint8_t* data, bool adc_cmd_pdu_ch8);

  // config detail: {'description': 'Control PDU Ch 7 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch7',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 14, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch7(uint8_t* data, bool adc_cmd_pdu_ch7);

  // config detail: {'description': 'Control PDU Ch 6 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch6',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 13, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch6(uint8_t* data, bool adc_cmd_pdu_ch6);

  // config detail: {'description': 'Control PDU Ch 5 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch5',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 12, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch5(uint8_t* data, bool adc_cmd_pdu_ch5);

  // config detail: {'description': 'Control PDU Ch 4 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch4',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 11, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch4(uint8_t* data, bool adc_cmd_pdu_ch4);

  // config detail: {'description': 'Control PDU Ch 3 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch3',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch3(uint8_t* data, bool adc_cmd_pdu_ch3);

  // config detail: {'description': 'Control PDU Ch 2 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch2',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 9, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch2(uint8_t* data, bool adc_cmd_pdu_ch2);

  // config detail: {'description': 'Control PDU Ch 1 (when override enabled)',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch1',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 8, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_pdu_ch1(uint8_t* data, bool adc_cmd_pdu_ch1);

  // config detail: {'description': 'Control hazard lights', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_HazardLights',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 28, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_hazardlights(uint8_t* data, bool adc_cmd_hazardlights);

  // config detail: {'description': 'Control high beam', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_HighBeam', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_highbeam(uint8_t* data, bool adc_cmd_highbeam);

  // config detail: {'description': 'Control low beam', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_LowBeam', 'is_signed_var':
  // False, 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order':
  // 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_lowbeam(uint8_t* data, bool adc_cmd_lowbeam);

  // config detail: {'description': 'Control horn', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_Horn', 'is_signed_var': False,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'intel',
  // 'physical_unit': 'T/F'}
  void set_p_adc_cmd_horn(uint8_t* data, bool adc_cmd_horn);

  // config detail: {'description': 'Requested turn signals', 'enum': {0:
  // 'ADC_CMD_TURNSIGNAL_NONE', 1: 'ADC_CMD_TURNSIGNAL_LEFT', 2:
  // 'ADC_CMD_TURNSIGNAL_RIGHT', 3: 'ADC_CMD_TURNSIGNAL_RESERVE'},
  // 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_TurnSignal', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_cmd_turnsignal(
      uint8_t* data,
      Adc_auxiliarycontrol_110::Adc_cmd_turnsignalType adc_cmd_turnsignal);

 private:
  int adc_auxcontrol_counter_;
  int adc_auxcontrol_checksum_;
  bool adc_cmd_inverter_controlenable_;
  bool adc_cmd_inverter_;
  int adc_cmd_wiper_;
  bool adc_cmd_pdu_controlenable_;
  bool adc_cmd_pdu_ch8_;
  bool adc_cmd_pdu_ch7_;
  bool adc_cmd_pdu_ch6_;
  bool adc_cmd_pdu_ch5_;
  bool adc_cmd_pdu_ch4_;
  bool adc_cmd_pdu_ch3_;
  bool adc_cmd_pdu_ch2_;
  bool adc_cmd_pdu_ch1_;
  bool adc_cmd_hazardlights_;
  bool adc_cmd_highbeam_;
  bool adc_cmd_lowbeam_;
  bool adc_cmd_horn_;
  Adc_auxiliarycontrol_110::Adc_cmd_turnsignalType adc_cmd_turnsignal_;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
