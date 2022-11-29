/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_vehicle_fault_response_201.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::drivers::canbus::Byte;

Vcuvehiclefaultresponse201::Vcuvehiclefaultresponse201() {}
const int32_t Vcuvehiclefaultresponse201::ID = 0x201;

void Vcuvehiclefaultresponse201::Parse(const std::uint8_t* bytes,
                                       int32_t length,
                                       Neolix_edu* chassis) const {
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_vehicle_error_indicationsvcu(
          vehicle_error_indicationsvcu(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_brake_system_errorehb(
          brake_system_errorehb(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_eps_error(eps_error(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_motor_error(motor_error(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_epb_error(epb_error(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_high_voltage_battery_errorbcu(
          high_voltage_battery_errorbcu(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_losscommuni(
          automode_exit_reason_losscommuni(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_reqsignalno(
          automode_exit_reason_reqsignalno(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_low_power(
          automode_exit_reason_low_power(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_highvolt(
          automode_exit_reason_highvolt(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_vehicle_flt(
          automode_exit_reason_vehicle_flt(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_press_emerg(
          automode_exit_reason_press_emerg(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_press_remot(
          automode_exit_reason_press_remot(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_automode_exit_reason_pdu_control(
          automode_exit_reason_pdu_control(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_vcu_faultrept_alivecounter(
          vcu_faultrept_alivecounter(bytes, length));
  chassis->mutable_vcu_vehicle_fault_response_201()
      ->set_vcu_faultrept_checksum(
          vcu_faultrept_checksum(bytes, length));
}

// config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level 2
// error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error', 'offset':
// 0.0, 'precision': 1.0, 'len': 4, 'name': 'vehicle_error_indicationsvcu',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 3, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclefaultresponse201::vehicle_error_indicationsvcu(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level 2
// error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error', 'offset':
// 0.0, 'precision': 1.0, 'len': 4, 'name': 'brake_system_errorehb',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclefaultresponse201::brake_system_errorehb(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level 2
// error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error', 'offset':
// 0.0, 'precision': 1.0, 'len': 4, 'name': 'eps_error', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 11, 'type': 'int', 'order': 'motorola',
// 'physical_unit': ''}
int Vcuvehiclefaultresponse201::eps_error(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level 2
// error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error', 'offset':
// 0.0, 'precision': 1.0, 'len': 4, 'name': 'motor_error', 'is_signed_var':
// False, 'physical_range': '[0|0]', 'bit': 15, 'type': 'int', 'order':
// 'motorola', 'physical_unit': ''}
int Vcuvehiclefaultresponse201::motor_error(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level 2
// error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error', 'offset':
// 0.0, 'precision': 1.0, 'len': 4, 'name': 'epb_error', 'is_signed_var': False,
// 'physical_range': '[0|0]', 'bit': 19, 'type': 'int', 'order': 'motorola',
// 'physical_unit': ''}
int Vcuvehiclefaultresponse201::epb_error(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0: no error;0x1: level 1 error;0x2: level 2
// error;0x3: level 3 error;0x4: level 4 error;0x5: level 5 error', 'offset':
// 0.0, 'precision': 1.0, 'len': 4, 'name': 'high_voltage_battery_errorbcu',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'int',
// 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclefaultresponse201::high_voltage_battery_errorbcu(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_losscommuni',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_losscommuni(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_reqsignalno',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 33, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_reqsignalno(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_low_power',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 34, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_low_power(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_highvolt',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 35, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_highvolt(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_vehicle_flt',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_vehicle_flt(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_press_emerg',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 37, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_press_emerg(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_press_remot',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 38, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_press_remot(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'description': '0x0:Normal;0x1:Failure', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'name': 'automode_exit_reason_pdu_control',
// 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'bool',
// 'order': 'motorola', 'physical_unit': ''}
bool Vcuvehiclefaultresponse201::automode_exit_reason_pdu_control(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'vcu_faultrept_alivecounter', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclefaultresponse201::vcu_faultrept_alivecounter(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_faultrept_checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Vcuvehiclefaultresponse201::vcu_faultrept_checksum(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
