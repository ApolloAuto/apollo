/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/minibus/protocol/controller_status_requset_18ff86a9.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

const int32_t Controllerstatusrequset18ff86a9::ID = 0x38ff86a9;

// public
Controllerstatusrequset18ff86a9::Controllerstatusrequset18ff86a9() { Reset(); }

uint32_t Controllerstatusrequset18ff86a9::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controllerstatusrequset18ff86a9::UpdateData(uint8_t* data) {
  set_p_sr_rtk_power(data, sr_rtk_power_);
  set_p_sr_fullview_power(data, sr_fullview_power_);
  set_p_sr_mobieye_power(data, sr_mobieye_power_);
  set_p_sr_mmradar_power(data, sr_mmradar_power_);
  set_p_sr_ultrasonicradar_power(data, sr_ultrasonicradar_power_);
  set_p_sr_bsdradar_power(data, sr_bsdradar_power_);
  set_p_sr_singlelindlidar_power(data, sr_singlelindlidar_power_);
  set_p_sr_16lidar_power(data, sr_16lidar_power_);
  set_p_sr_horn(data, sr_horn_);
  set_p_sr_fluorescentlamps(data, sr_fluorescentlamps_);
  set_p_sr_rooflight(data, sr_rooflight_);
  set_p_sr_minillight(data, sr_minillight_);
  set_p_sr_breaklight(data, sr_breaklight_);
  set_p_sr_turnright(data, sr_turnright_);
  set_p_sr_turnleft(data, sr_turnleft_);
  set_p_sr_reversinglight(data, sr_reversinglight_);
  set_p_sr_lowbeam(data, sr_lowbeam_);
  set_p_sr_fdoor(data, sr_fdoor_);
  set_p_sr_gear_status(data, sr_gear_status_);
  set_p_sr_vehiclemoveing_status(data, sr_vehiclemoveing_status_);
  set_p_sr_drive_status(data, sr_drive_status_);
  set_p_sr_inertialnavigation_status(data, sr_inertialnavigation_status_);
  set_p_sr_rtk_status(data, sr_rtk_status_);
}

void Controllerstatusrequset18ff86a9::Reset() {
  // TODO(All) :  you should check this manually
  sr_rtk_power_ = 0;
  sr_fullview_power_ = 0;
  sr_mobieye_power_ = 0;
  sr_mmradar_power_ = 0;
  sr_ultrasonicradar_power_ = 0;
  sr_bsdradar_power_ = 0;
  sr_singlelindlidar_power_ = 0;
  sr_16lidar_power_ = 0;
  sr_horn_ = Controller_status_requset_18ff86a9::SR_HORN_OFF;
  sr_fluorescentlamps_ = 0;
  sr_rooflight_ = 0;
  sr_minillight_ = 0;
  sr_breaklight_ = Controller_status_requset_18ff86a9::SR_BREAKLIGHT_OFF;
  sr_turnright_ = Controller_status_requset_18ff86a9::SR_TURNRIGHT_OFF;
  sr_turnleft_ = Controller_status_requset_18ff86a9::SR_TURNLEFT_OFF;
  sr_reversinglight_ = 0;
  sr_lowbeam_ = Controller_status_requset_18ff86a9::SR_LOWBEAM_OFF;
  sr_fdoor_ = Controller_status_requset_18ff86a9::SR_FDOOR_NO_ACTION;
  sr_gear_status_ = 0;
  sr_vehiclemoveing_status_ = 0;
  sr_drive_status_ = 0;
  sr_inertialnavigation_status_ = 0;
  sr_rtk_status_ = 0;
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_rtk_power(int sr_rtk_power) {
  sr_rtk_power_ = sr_rtk_power;
  return this;
}

// config detail: {'bit': 54, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_RTK_Power', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_rtk_power(uint8_t* data,
                                                         int sr_rtk_power) {
  sr_rtk_power = ProtocolData::BoundedValue(0, 0, sr_rtk_power);
  int x = sr_rtk_power;

  Byte to_set(data + 6);
  to_set.set_value(x, 6, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_fullview_power(int sr_fullview_power) {
  sr_fullview_power_ = sr_fullview_power;
  return this;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_FullView_Power', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_fullview_power(
    uint8_t* data, int sr_fullview_power) {
  sr_fullview_power = ProtocolData::BoundedValue(0, 0, sr_fullview_power);
  int x = sr_fullview_power;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_mobieye_power(int sr_mobieye_power) {
  sr_mobieye_power_ = sr_mobieye_power;
  return this;
}

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_Mobieye_Power', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_mobieye_power(
    uint8_t* data, int sr_mobieye_power) {
  sr_mobieye_power = ProtocolData::BoundedValue(0, 0, sr_mobieye_power);
  int x = sr_mobieye_power;

  Byte to_set(data + 6);
  to_set.set_value(x, 2, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_mmradar_power(int sr_mmradar_power) {
  sr_mmradar_power_ = sr_mmradar_power;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_MmRadar_Power', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_mmradar_power(
    uint8_t* data, int sr_mmradar_power) {
  sr_mmradar_power = ProtocolData::BoundedValue(0, 0, sr_mmradar_power);
  int x = sr_mmradar_power;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_ultrasonicradar_power(
    int sr_ultrasonicradar_power) {
  sr_ultrasonicradar_power_ = sr_ultrasonicradar_power;
  return this;
}

// config detail: {'bit': 46, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_UltrasonicRadar_Power', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_ultrasonicradar_power(
    uint8_t* data, int sr_ultrasonicradar_power) {
  sr_ultrasonicradar_power =
      ProtocolData::BoundedValue(0, 0, sr_ultrasonicradar_power);
  int x = sr_ultrasonicradar_power;

  Byte to_set(data + 5);
  to_set.set_value(x, 6, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_bsdradar_power(int sr_bsdradar_power) {
  sr_bsdradar_power_ = sr_bsdradar_power;
  return this;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_BSDRadar_Power', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_bsdradar_power(
    uint8_t* data, int sr_bsdradar_power) {
  sr_bsdradar_power = ProtocolData::BoundedValue(0, 0, sr_bsdradar_power);
  int x = sr_bsdradar_power;

  Byte to_set(data + 5);
  to_set.set_value(x, 4, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_singlelindlidar_power(
    int sr_singlelindlidar_power) {
  sr_singlelindlidar_power_ = sr_singlelindlidar_power;
  return this;
}

// config detail: {'bit': 42, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_SingleLindLiDar_Power', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_singlelindlidar_power(
    uint8_t* data, int sr_singlelindlidar_power) {
  sr_singlelindlidar_power =
      ProtocolData::BoundedValue(0, 0, sr_singlelindlidar_power);
  int x = sr_singlelindlidar_power;

  Byte to_set(data + 5);
  to_set.set_value(x, 2, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_16lidar_power(int sr_16lidar_power) {
  sr_16lidar_power_ = sr_16lidar_power;
  return this;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_16LiDar_Power', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_16lidar_power(
    uint8_t* data, int sr_16lidar_power) {
  sr_16lidar_power = ProtocolData::BoundedValue(0, 0, sr_16lidar_power);
  int x = sr_16lidar_power;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 2);
}

Controllerstatusrequset18ff86a9* Controllerstatusrequset18ff86a9::set_sr_horn(
    Controller_status_requset_18ff86a9::Sr_hornType sr_horn) {
  sr_horn_ = sr_horn;
  return this;
}

// config detail: {'bit': 36, 'enum': {0: 'SR_HORN_OFF', 1: 'SR_HORN_ON'},
// 'is_signed_var': False, 'len': 2, 'name': 'SR_Horn', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'enum'}
void Controllerstatusrequset18ff86a9::set_p_sr_horn(
    uint8_t* data, Controller_status_requset_18ff86a9::Sr_hornType sr_horn) {
  int x = sr_horn;

  Byte to_set(data + 4);
  to_set.set_value(x, 4, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_fluorescentlamps(
    int sr_fluorescentlamps) {
  sr_fluorescentlamps_ = sr_fluorescentlamps;
  return this;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_FluorescentLamps', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_fluorescentlamps(
    uint8_t* data, int sr_fluorescentlamps) {
  sr_fluorescentlamps = ProtocolData::BoundedValue(0, 0, sr_fluorescentlamps);
  int x = sr_fluorescentlamps;

  Byte to_set(data + 4);
  to_set.set_value(x, 2, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_rooflight(int sr_rooflight) {
  sr_rooflight_ = sr_rooflight;
  return this;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_Rooflight', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_rooflight(uint8_t* data,
                                                         int sr_rooflight) {
  sr_rooflight = ProtocolData::BoundedValue(0, 0, sr_rooflight);
  int x = sr_rooflight;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_minillight(int sr_minillight) {
  sr_minillight_ = sr_minillight;
  return this;
}

// config detail: {'bit': 30, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_MinilLight', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_minillight(uint8_t* data,
                                                          int sr_minillight) {
  sr_minillight = ProtocolData::BoundedValue(0, 0, sr_minillight);
  int x = sr_minillight;

  Byte to_set(data + 3);
  to_set.set_value(x, 6, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_breaklight(
    Controller_status_requset_18ff86a9::Sr_breaklightType sr_breaklight) {
  sr_breaklight_ = sr_breaklight;
  return this;
}

// config detail: {'bit': 28, 'enum': {0: 'SR_BREAKLIGHT_OFF', 1:
// 'SR_BREAKLIGHT_ON'}, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_BreakLight', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Controllerstatusrequset18ff86a9::set_p_sr_breaklight(
    uint8_t* data,
    Controller_status_requset_18ff86a9::Sr_breaklightType sr_breaklight) {
  int x = sr_breaklight;

  Byte to_set(data + 3);
  to_set.set_value(x, 4, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_turnright(
    Controller_status_requset_18ff86a9::Sr_turnrightType sr_turnright) {
  sr_turnright_ = sr_turnright;
  return this;
}

// config detail: {'bit': 26, 'enum': {0: 'SR_TURNRIGHT_OFF', 1:
// 'SR_TURNRIGHT_ON'}, 'is_signed_var': False, 'len': 2, 'name': 'SR_TurnRight',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'enum'}
void Controllerstatusrequset18ff86a9::set_p_sr_turnright(
    uint8_t* data,
    Controller_status_requset_18ff86a9::Sr_turnrightType sr_turnright) {
  int x = sr_turnright;

  Byte to_set(data + 3);
  to_set.set_value(x, 2, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_turnleft(
    Controller_status_requset_18ff86a9::Sr_turnleftType sr_turnleft) {
  sr_turnleft_ = sr_turnleft;
  return this;
}

// config detail: {'bit': 24, 'enum': {0: 'SR_TURNLEFT_OFF', 1:
// 'SR_TURNLEFT_ON'}, 'is_signed_var': False, 'len': 2, 'name': 'SR_TurnLeft',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'enum'}
void Controllerstatusrequset18ff86a9::set_p_sr_turnleft(
    uint8_t* data,
    Controller_status_requset_18ff86a9::Sr_turnleftType sr_turnleft) {
  int x = sr_turnleft;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_reversinglight(int sr_reversinglight) {
  sr_reversinglight_ = sr_reversinglight;
  return this;
}

// config detail: {'bit': 22, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_ReversingLight', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_reversinglight(
    uint8_t* data, int sr_reversinglight) {
  sr_reversinglight = ProtocolData::BoundedValue(0, 0, sr_reversinglight);
  int x = sr_reversinglight;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_lowbeam(
    Controller_status_requset_18ff86a9::Sr_lowbeamType sr_lowbeam) {
  sr_lowbeam_ = sr_lowbeam;
  return this;
}

// config detail: {'bit': 20, 'enum': {0: 'SR_LOWBEAM_OFF', 1: 'SR_LOWBEAM_ON'},
// 'is_signed_var': False, 'len': 2, 'name': 'SR_Lowbeam', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
void Controllerstatusrequset18ff86a9::set_p_sr_lowbeam(
    uint8_t* data,
    Controller_status_requset_18ff86a9::Sr_lowbeamType sr_lowbeam) {
  int x = sr_lowbeam;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 2);
}

Controllerstatusrequset18ff86a9* Controllerstatusrequset18ff86a9::set_sr_fdoor(
    Controller_status_requset_18ff86a9::Sr_fdoorType sr_fdoor) {
  sr_fdoor_ = sr_fdoor;
  return this;
}

// config detail: {'bit': 18, 'enum': {0: 'SR_FDOOR_NO_ACTION', 1:
// 'SR_FDOOR_DOOR_OPEN', 2: 'SR_FDOOR_DOOR_CLOSE'}, 'is_signed_var': False,
// 'len': 2, 'name': 'SR_Fdoor', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
void Controllerstatusrequset18ff86a9::set_p_sr_fdoor(
    uint8_t* data, Controller_status_requset_18ff86a9::Sr_fdoorType sr_fdoor) {
  int x = sr_fdoor;

  Byte to_set(data + 2);
  to_set.set_value(x, 2, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_gear_status(int sr_gear_status) {
  sr_gear_status_ = sr_gear_status;
  return this;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 2, 'name':
// 'SR_Gear_Status', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_gear_status(uint8_t* data,
                                                           int sr_gear_status) {
  sr_gear_status = ProtocolData::BoundedValue(0, 0, sr_gear_status);
  int x = sr_gear_status;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 2);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_vehiclemoveing_status(
    int sr_vehiclemoveing_status) {
  sr_vehiclemoveing_status_ = sr_vehiclemoveing_status;
  return this;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 4, 'name':
// 'SR_VehicleMoveing_Status', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_vehiclemoveing_status(
    uint8_t* data, int sr_vehiclemoveing_status) {
  sr_vehiclemoveing_status =
      ProtocolData::BoundedValue(0, 0, sr_vehiclemoveing_status);
  int x = sr_vehiclemoveing_status;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 4);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_drive_status(int sr_drive_status) {
  sr_drive_status_ = sr_drive_status;
  return this;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 4, 'name':
// 'SR_Drive_Status', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_drive_status(
    uint8_t* data, int sr_drive_status) {
  sr_drive_status = ProtocolData::BoundedValue(0, 0, sr_drive_status);
  int x = sr_drive_status;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 4);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_inertialnavigation_status(
    int sr_inertialnavigation_status) {
  sr_inertialnavigation_status_ = sr_inertialnavigation_status;
  return this;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 4, 'name':
// 'SR_InertialNavigation_Status', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_inertialnavigation_status(
    uint8_t* data, int sr_inertialnavigation_status) {
  sr_inertialnavigation_status =
      ProtocolData::BoundedValue(0, 0, sr_inertialnavigation_status);
  int x = sr_inertialnavigation_status;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Controllerstatusrequset18ff86a9*
Controllerstatusrequset18ff86a9::set_sr_rtk_status(int sr_rtk_status) {
  sr_rtk_status_ = sr_rtk_status;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name':
// 'SR_RTK_Status', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Controllerstatusrequset18ff86a9::set_p_sr_rtk_status(uint8_t* data,
                                                          int sr_rtk_status) {
  sr_rtk_status = ProtocolData::BoundedValue(0, 0, sr_rtk_status);
  int x = sr_rtk_status;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 4);
}

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
