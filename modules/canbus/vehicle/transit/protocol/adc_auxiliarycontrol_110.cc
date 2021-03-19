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

#include "modules/canbus/vehicle/transit/protocol/adc_auxiliarycontrol_110.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

const int32_t Adcauxiliarycontrol110::ID = 0x110;

// public
Adcauxiliarycontrol110::Adcauxiliarycontrol110() { Reset(); }

uint32_t Adcauxiliarycontrol110::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Adcauxiliarycontrol110::UpdateData(uint8_t* data) {
  set_p_adc_auxcontrol_counter(data, adc_auxcontrol_counter_);
  set_p_adc_auxcontrol_checksum(data, adc_auxcontrol_checksum_);
  set_p_adc_cmd_inverter_controlenable(data, adc_cmd_inverter_controlenable_);
  set_p_adc_cmd_inverter(data, adc_cmd_inverter_);
  set_p_adc_cmd_wiper(data, adc_cmd_wiper_);
  set_p_adc_cmd_pdu_controlenable(data, adc_cmd_pdu_controlenable_);
  set_p_adc_cmd_pdu_ch8(data, adc_cmd_pdu_ch8_);
  set_p_adc_cmd_pdu_ch7(data, adc_cmd_pdu_ch7_);
  set_p_adc_cmd_pdu_ch6(data, adc_cmd_pdu_ch6_);
  set_p_adc_cmd_pdu_ch5(data, adc_cmd_pdu_ch5_);
  set_p_adc_cmd_pdu_ch4(data, adc_cmd_pdu_ch4_);
  set_p_adc_cmd_pdu_ch3(data, adc_cmd_pdu_ch3_);
  set_p_adc_cmd_pdu_ch2(data, adc_cmd_pdu_ch2_);
  set_p_adc_cmd_pdu_ch1(data, adc_cmd_pdu_ch1_);
  set_p_adc_cmd_hazardlights(data, adc_cmd_hazardlights_);
  set_p_adc_cmd_highbeam(data, adc_cmd_highbeam_);
  set_p_adc_cmd_lowbeam(data, adc_cmd_lowbeam_);
  set_p_adc_cmd_horn(data, adc_cmd_horn_);
  set_p_adc_cmd_turnsignal(data, adc_cmd_turnsignal_);
}

void Adcauxiliarycontrol110::Reset() {
  // TODO(All) :  you should check this manually
  adc_auxcontrol_counter_ = 0;
  adc_auxcontrol_checksum_ = 0;
  adc_cmd_inverter_controlenable_ = false;
  adc_cmd_inverter_ = false;
  adc_cmd_wiper_ = 0;
  adc_cmd_pdu_controlenable_ = false;
  adc_cmd_pdu_ch8_ = false;
  adc_cmd_pdu_ch7_ = false;
  adc_cmd_pdu_ch6_ = false;
  adc_cmd_pdu_ch5_ = false;
  adc_cmd_pdu_ch4_ = false;
  adc_cmd_pdu_ch3_ = false;
  adc_cmd_pdu_ch2_ = false;
  adc_cmd_pdu_ch1_ = false;
  adc_cmd_hazardlights_ = false;
  adc_cmd_highbeam_ = false;
  adc_cmd_lowbeam_ = false;
  adc_cmd_horn_ = false;
  adc_cmd_turnsignal_ = Adc_auxiliarycontrol_110::ADC_CMD_TURNSIGNAL_NONE;
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_auxcontrol_counter(
    int adc_auxcontrol_counter) {
  adc_auxcontrol_counter_ = adc_auxcontrol_counter;
  return this;
}

// config detail: {'description': 'Aux control heartbeat counter', 'offset':
// 0.0, 'precision': 1.0, 'len': 2, 'name': 'ADC_AuxControl_Counter',
// 'is_signed_var': False, 'physical_range': '[0|3]', 'bit': 54, 'type': 'int',
// 'order': 'intel', 'physical_unit': ''}
void Adcauxiliarycontrol110::set_p_adc_auxcontrol_counter(
    uint8_t* data, int adc_auxcontrol_counter) {
  adc_auxcontrol_counter =
      ProtocolData::BoundedValue(0, 3, adc_auxcontrol_counter);
  uint8_t x = static_cast<uint8_t>(adc_auxcontrol_counter);

  Byte to_set(data + 6);
  to_set.set_value(x, 6, 2);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_auxcontrol_checksum(
    int adc_auxcontrol_checksum) {
  adc_auxcontrol_checksum_ = adc_auxcontrol_checksum;
  return this;
}

// config detail: {'description': 'Aux control checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'ADC_AuxControl_Checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
void Adcauxiliarycontrol110::set_p_adc_auxcontrol_checksum(
    uint8_t* data, int adc_auxcontrol_checksum) {
  adc_auxcontrol_checksum =
      ProtocolData::BoundedValue(0, 255, adc_auxcontrol_checksum);
  uint8_t x = static_cast<uint8_t>(adc_auxcontrol_checksum);

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

Adcauxiliarycontrol110*
Adcauxiliarycontrol110::set_adc_cmd_inverter_controlenable(
    bool adc_cmd_inverter_controlenable) {
  adc_cmd_inverter_controlenable_ = adc_cmd_inverter_controlenable;
  return this;
}

// config detail: {'description': 'Control inverter override (default ON if not
// overridden)', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
// 'ADC_CMD_Inverter_ControlEnable', 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_inverter_controlenable(
    uint8_t* data, bool adc_cmd_inverter_controlenable) {
  uint8_t x = adc_cmd_inverter_controlenable;

  Byte to_set(data + 0);
  to_set.set_value(x, 1, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_inverter(
    bool adc_cmd_inverter) {
  adc_cmd_inverter_ = adc_cmd_inverter;
  return this;
}

// config detail: {'description': 'Control inverter', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_Inverter', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'intel',
// 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_inverter(uint8_t* data,
                                                    bool adc_cmd_inverter) {
  uint8_t x = adc_cmd_inverter;

  Byte to_set(data + 0);
  to_set.set_value(x, 2, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_wiper(
    int adc_cmd_wiper) {
  adc_cmd_wiper_ = adc_cmd_wiper;
  return this;
}

// config detail: {'description': '(Reserved) Control wiper', 'offset': 0.0,
// 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_Wiper', 'is_signed_var': False,
// 'physical_range': '[0|3]', 'bit': 4, 'type': 'int', 'order': 'intel',
// 'physical_unit': ''}
void Adcauxiliarycontrol110::set_p_adc_cmd_wiper(uint8_t* data,
                                                 int adc_cmd_wiper) {
  adc_cmd_wiper = ProtocolData::BoundedValue(0, 3, adc_cmd_wiper);
  uint8_t x = static_cast<uint8_t>(adc_cmd_wiper);

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 2);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_controlenable(
    bool adc_cmd_pdu_controlenable) {
  adc_cmd_pdu_controlenable_ = adc_cmd_pdu_controlenable;
  return this;
}

// config detail: {'description': 'PDU Control Override (all channels default ON
// if not overridden)', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name':
// 'ADC_CMD_PDU_ControlEnable', 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_controlenable(
    uint8_t* data, bool adc_cmd_pdu_controlenable) {
  uint8_t x = adc_cmd_pdu_controlenable;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch8(
    bool adc_cmd_pdu_ch8) {
  adc_cmd_pdu_ch8_ = adc_cmd_pdu_ch8;
  return this;
}

// config detail: {'description': 'Control PDU Ch 8 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch8',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 15, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch8(uint8_t* data,
                                                   bool adc_cmd_pdu_ch8) {
  uint8_t x = adc_cmd_pdu_ch8;

  Byte to_set(data + 1);
  to_set.set_value(x, 7, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch7(
    bool adc_cmd_pdu_ch7) {
  adc_cmd_pdu_ch7_ = adc_cmd_pdu_ch7;
  return this;
}

// config detail: {'description': 'Control PDU Ch 7 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch7',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 14, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch7(uint8_t* data,
                                                   bool adc_cmd_pdu_ch7) {
  uint8_t x = adc_cmd_pdu_ch7;

  Byte to_set(data + 1);
  to_set.set_value(x, 6, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch6(
    bool adc_cmd_pdu_ch6) {
  adc_cmd_pdu_ch6_ = adc_cmd_pdu_ch6;
  return this;
}

// config detail: {'description': 'Control PDU Ch 6 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch6',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 13, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch6(uint8_t* data,
                                                   bool adc_cmd_pdu_ch6) {
  uint8_t x = adc_cmd_pdu_ch6;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch5(
    bool adc_cmd_pdu_ch5) {
  adc_cmd_pdu_ch5_ = adc_cmd_pdu_ch5;
  return this;
}

// config detail: {'description': 'Control PDU Ch 5 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch5',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 12, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch5(uint8_t* data,
                                                   bool adc_cmd_pdu_ch5) {
  uint8_t x = adc_cmd_pdu_ch5;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch4(
    bool adc_cmd_pdu_ch4) {
  adc_cmd_pdu_ch4_ = adc_cmd_pdu_ch4;
  return this;
}

// config detail: {'description': 'Control PDU Ch 4 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch4',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 11, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch4(uint8_t* data,
                                                   bool adc_cmd_pdu_ch4) {
  uint8_t x = adc_cmd_pdu_ch4;

  Byte to_set(data + 1);
  to_set.set_value(x, 3, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch3(
    bool adc_cmd_pdu_ch3) {
  adc_cmd_pdu_ch3_ = adc_cmd_pdu_ch3;
  return this;
}

// config detail: {'description': 'Control PDU Ch 3 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch3',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 10, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch3(uint8_t* data,
                                                   bool adc_cmd_pdu_ch3) {
  uint8_t x = adc_cmd_pdu_ch3;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch2(
    bool adc_cmd_pdu_ch2) {
  adc_cmd_pdu_ch2_ = adc_cmd_pdu_ch2;
  return this;
}

// config detail: {'description': 'Control PDU Ch 2 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch2',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 9, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch2(uint8_t* data,
                                                   bool adc_cmd_pdu_ch2) {
  uint8_t x = adc_cmd_pdu_ch2;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_pdu_ch1(
    bool adc_cmd_pdu_ch1) {
  adc_cmd_pdu_ch1_ = adc_cmd_pdu_ch1;
  return this;
}

// config detail: {'description': 'Control PDU Ch 1 (when override enabled)',
// 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_PDU_Ch1',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 8, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_pdu_ch1(uint8_t* data,
                                                   bool adc_cmd_pdu_ch1) {
  uint8_t x = adc_cmd_pdu_ch1;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_hazardlights(
    bool adc_cmd_hazardlights) {
  adc_cmd_hazardlights_ = adc_cmd_hazardlights;
  return this;
}

// config detail: {'description': 'Control hazard lights', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_HazardLights', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 28, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_hazardlights(
    uint8_t* data, bool adc_cmd_hazardlights) {
  uint8_t x = adc_cmd_hazardlights;

  Byte to_set(data + 3);
  to_set.set_value(x, 4, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_highbeam(
    bool adc_cmd_highbeam) {
  adc_cmd_highbeam_ = adc_cmd_highbeam;
  return this;
}

// config detail: {'description': 'Control high beam', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_HighBeam', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 27, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_highbeam(uint8_t* data,
                                                    bool adc_cmd_highbeam) {
  uint8_t x = adc_cmd_highbeam;

  Byte to_set(data + 3);
  to_set.set_value(x, 3, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_lowbeam(
    bool adc_cmd_lowbeam) {
  adc_cmd_lowbeam_ = adc_cmd_lowbeam;
  return this;
}

// config detail: {'description': 'Control low beam', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_LowBeam', 'is_signed_var':
// False, 'physical_range': '[0|1]', 'bit': 26, 'type': 'bool', 'order':
// 'intel', 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_lowbeam(uint8_t* data,
                                                   bool adc_cmd_lowbeam) {
  uint8_t x = adc_cmd_lowbeam;

  Byte to_set(data + 3);
  to_set.set_value(x, 2, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_horn(
    bool adc_cmd_horn) {
  adc_cmd_horn_ = adc_cmd_horn;
  return this;
}

// config detail: {'description': 'Control horn', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_Horn', 'is_signed_var': False,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'intel',
// 'physical_unit': 'T/F'}
void Adcauxiliarycontrol110::set_p_adc_cmd_horn(uint8_t* data,
                                                bool adc_cmd_horn) {
  uint8_t x = adc_cmd_horn;

  Byte to_set(data + 0);
  to_set.set_value(x, 3, 1);
}

Adcauxiliarycontrol110* Adcauxiliarycontrol110::set_adc_cmd_turnsignal(
    Adc_auxiliarycontrol_110::Adc_cmd_turnsignalType adc_cmd_turnsignal) {
  adc_cmd_turnsignal_ = adc_cmd_turnsignal;
  return this;
}

// config detail: {'description': 'Requested turn signals', 'enum': {0:
// 'ADC_CMD_TURNSIGNAL_NONE', 1: 'ADC_CMD_TURNSIGNAL_LEFT', 2:
// 'ADC_CMD_TURNSIGNAL_RIGHT', 3: 'ADC_CMD_TURNSIGNAL_RESERVE'},
// 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_TurnSignal', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 24, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
void Adcauxiliarycontrol110::set_p_adc_cmd_turnsignal(
    uint8_t* data,
    Adc_auxiliarycontrol_110::Adc_cmd_turnsignalType adc_cmd_turnsignal) {
  uint8_t x = adc_cmd_turnsignal;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 2);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
