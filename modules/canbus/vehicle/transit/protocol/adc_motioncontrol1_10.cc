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

#include "modules/canbus/vehicle/transit/protocol/adc_motioncontrol1_10.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::drivers::canbus::Byte;

const int32_t Adcmotioncontrol110::ID = 0x10;

// public
Adcmotioncontrol110::Adcmotioncontrol110() { Reset(); }

uint32_t Adcmotioncontrol110::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Adcmotioncontrol110::UpdateData(uint8_t* data) {
  set_p_adc_cmd_steerwheelangle(data, adc_cmd_steerwheelangle_);
  set_p_adc_cmd_steeringcontrolmode(data, adc_cmd_steeringcontrolmode_);
  set_p_adc_cmd_parkingbrake(data, adc_cmd_parkingbrake_);
  set_p_adc_cmd_gear(data, adc_cmd_gear_);
  set_p_adc_motioncontrol1_checksum(data, adc_motioncontrol1_checksum_);
  set_p_adc_cmd_brakepercentage(data, adc_cmd_brakepercentage_);
  set_p_adc_cmd_throttleposition(data, adc_cmd_throttleposition_);
  set_p_adc_motioncontrol1_counter(data, adc_motioncontrol1_counter_);
  set_p_adc_cmd_autonomyrequest(data, adc_cmd_autonomyrequest_);
  set_p_adc_cmd_longitudinalcontrolmode(data, adc_cmd_longitudinalcontrolmode_);
}

void Adcmotioncontrol110::Reset() {
  // TODO(All) :  you should check this manually
  adc_cmd_steerwheelangle_ = 0.0;
  adc_cmd_steeringcontrolmode_ =
      Adc_motioncontrol1_10::ADC_CMD_STEERINGCONTROLMODE_NONE;
  adc_cmd_parkingbrake_ = false;
  adc_cmd_gear_ = Adc_motioncontrol1_10::ADC_CMD_GEAR_P_PARK;
  adc_motioncontrol1_checksum_ = 0;
  adc_cmd_brakepercentage_ = 0.0;
  adc_cmd_throttleposition_ = 0.0;
  adc_motioncontrol1_counter_ = 0;
  adc_cmd_autonomyrequest_ =
      Adc_motioncontrol1_10::ADC_CMD_AUTONOMYREQUEST_AUTONOMY_NOT_REQUESTED;
  adc_cmd_longitudinalcontrolmode_ =
      Adc_motioncontrol1_10::ADC_CMD_LONGITUDINALCONTROLMODE_NONE;
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_steerwheelangle(
    double adc_cmd_steerwheelangle) {
  adc_cmd_steerwheelangle_ = adc_cmd_steerwheelangle;
  return this;
}

// config detail: {'description': 'Setpoint for steering wheel angle. Positive
// for CW', 'offset': 0.0, 'precision': -0.05, 'len': 16, 'name':
// 'ADC_CMD_SteerWheelAngle', 'is_signed_var': True, 'physical_range':
// '[-1638.4|1638.35]', 'bit': 27, 'type': 'double', 'order': 'intel',
// 'physical_unit': 'deg'}
void Adcmotioncontrol110::set_p_adc_cmd_steerwheelangle(
    uint8_t* data, double adc_cmd_steerwheelangle) {
  adc_cmd_steerwheelangle =
      ProtocolData::BoundedValue(-1638.4, 1638.35, adc_cmd_steerwheelangle);
  int x = static_cast<int>(adc_cmd_steerwheelangle / -0.050000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x1F);
  Byte to_set0(data + 3);
  to_set0.set_value(t, 3, 5);
  x >>= 5;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0x7);
  Byte to_set2(data + 5);
  to_set2.set_value(t, 0, 3);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_steeringcontrolmode(
    Adc_motioncontrol1_10::Adc_cmd_steeringcontrolmodeType
        adc_cmd_steeringcontrolmode) {
  adc_cmd_steeringcontrolmode_ = adc_cmd_steeringcontrolmode;
  return this;
}

// config detail: {'description': 'Select steering control mode', 'enum': {0:
// 'ADC_CMD_STEERINGCONTROLMODE_NONE', 1: 'ADC_CMD_STEERINGCONTROLMODE_ANGLE',
// 2: 'ADC_CMD_STEERINGCONTROLMODE_RESERVED_CURVATURE', 3:
// 'ADC_CMD_STEERINGCONTROLMODE_RESERVED'}, 'precision': 1.0, 'len': 2, 'name':
// 'ADC_CMD_SteeringControlMode', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
void Adcmotioncontrol110::set_p_adc_cmd_steeringcontrolmode(
    uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_steeringcontrolmodeType
                       adc_cmd_steeringcontrolmode) {
  int x = adc_cmd_steeringcontrolmode;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 4, 2);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_parkingbrake(
    bool adc_cmd_parkingbrake) {
  adc_cmd_parkingbrake_ = adc_cmd_parkingbrake;
  return this;
}

// config detail: {'description': '(Reserved) Control parking brake', 'offset':
// 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_ParkingBrake',
// 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 53, 'type': 'bool',
// 'order': 'intel', 'physical_unit': 'T/F'}
void Adcmotioncontrol110::set_p_adc_cmd_parkingbrake(
    uint8_t* data, bool adc_cmd_parkingbrake) {
  int x = adc_cmd_parkingbrake;

  Byte to_set(data + 6);
  to_set.set_value(static_cast<uint8_t>(x), 5, 1);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_gear(
    Adc_motioncontrol1_10::Adc_cmd_gearType adc_cmd_gear) {
  adc_cmd_gear_ = adc_cmd_gear;
  return this;
}

// config detail: {'description': 'Transmission control - only used in direct
// longitudinal control', 'enum': {0: 'ADC_CMD_GEAR_P_PARK', 1:
// 'ADC_CMD_GEAR_D_DRIVE', 2: 'ADC_CMD_GEAR_N_NEUTRAL', 3:
// 'ADC_CMD_GEAR_R_REVERSE'}, 'precision': 1.0, 'len': 3, 'name':
// 'ADC_CMD_Gear', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|7]', 'bit': 50, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adcmotioncontrol110::set_p_adc_cmd_gear(
    uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_gearType adc_cmd_gear) {
  int x = adc_cmd_gear;

  Byte to_set(data + 6);
  to_set.set_value(static_cast<uint8_t>(x), 2, 3);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_motioncontrol1_checksum(
    int adc_motioncontrol1_checksum) {
  adc_motioncontrol1_checksum_ = adc_motioncontrol1_checksum;
  return this;
}

// config detail: {'description': 'Motion Control 1 checksum', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'name': 'ADC_MotionControl1_Checksum',
// 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
void Adcmotioncontrol110::set_p_adc_motioncontrol1_checksum(
    uint8_t* data, int adc_motioncontrol1_checksum) {
  adc_motioncontrol1_checksum =
      ProtocolData::BoundedValue(0, 255, adc_motioncontrol1_checksum);
  int x = adc_motioncontrol1_checksum;

  Byte to_set(data + 7);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_brakepercentage(
    double adc_cmd_brakepercentage) {
  adc_cmd_brakepercentage_ = adc_cmd_brakepercentage;
  return this;
}

// config detail: {'description': 'Brake pressure for direct longitudinal
// control', 'offset': 0.0, 'precision': 0.0556, 'len': 11, 'name':
// 'ADC_CMD_BrakePercentage', 'is_signed_var': False, 'physical_range':
// '[0|113.8132]', 'bit': 6, 'type': 'double', 'order': 'intel',
// 'physical_unit': '%'}
void Adcmotioncontrol110::set_p_adc_cmd_brakepercentage(
    uint8_t* data, double adc_cmd_brakepercentage) {
  adc_cmd_brakepercentage =
      ProtocolData::BoundedValue(0.0, 113.8132, adc_cmd_brakepercentage);
  int x = static_cast<int>(adc_cmd_brakepercentage / 0.055600);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x3);
  Byte to_set0(data + 0);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0x1);
  Byte to_set2(data + 2);
  to_set2.set_value(t, 0, 1);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_throttleposition(
    double adc_cmd_throttleposition) {
  adc_cmd_throttleposition_ = adc_cmd_throttleposition;
  return this;
}

// config detail: {'description': 'Throttle pedal position percentage for direct
// longitudinal control', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
// 'ADC_CMD_ThrottlePosition', 'is_signed_var': False, 'physical_range':
// '[0|100]', 'bit': 17, 'type': 'double', 'order': 'intel', 'physical_unit':
// '%'}
void Adcmotioncontrol110::set_p_adc_cmd_throttleposition(
    uint8_t* data, double adc_cmd_throttleposition) {
  adc_cmd_throttleposition =
      ProtocolData::BoundedValue(0.0, 100.0, adc_cmd_throttleposition);
  int x = static_cast<int>(adc_cmd_throttleposition / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x7F);
  Byte to_set0(data + 2);
  to_set0.set_value(t, 1, 7);
  x >>= 7;

  t = static_cast<uint8_t>(x & 0x7);
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 3);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_motioncontrol1_counter(
    int adc_motioncontrol1_counter) {
  adc_motioncontrol1_counter_ = adc_motioncontrol1_counter;
  return this;
}

// config detail: {'description': 'Motion control 1 Heartbeat counter',
// 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
// 'ADC_MotionControl1_Counter', 'is_signed_var': False, 'physical_range':
// '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Adcmotioncontrol110::set_p_adc_motioncontrol1_counter(
    uint8_t* data, int adc_motioncontrol1_counter) {
  adc_motioncontrol1_counter =
      ProtocolData::BoundedValue(0, 3, adc_motioncontrol1_counter);
  int x = adc_motioncontrol1_counter;

  Byte to_set(data + 6);
  to_set.set_value(static_cast<uint8_t>(x), 6, 2);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_autonomyrequest(
    Adc_motioncontrol1_10::Adc_cmd_autonomyrequestType
        adc_cmd_autonomyrequest) {
  adc_cmd_autonomyrequest_ = adc_cmd_autonomyrequest;
  return this;
}

// config detail: {'description': 'Request from ADC to LLC for autonomy',
// 'enum': {0: 'ADC_CMD_AUTONOMYREQUEST_AUTONOMY_NOT_REQUESTED', 1:
// 'ADC_CMD_AUTONOMYREQUEST_AUTONOMY_REQUESTED', 2:
// 'ADC_CMD_AUTONOMYREQUEST_RESERVED0', 3: 'ADC_CMD_AUTONOMYREQUEST_RESERVED1'},
// 'precision': 1.0, 'len': 2, 'name': 'ADC_CMD_AutonomyRequest',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Adcmotioncontrol110::set_p_adc_cmd_autonomyrequest(
    uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_autonomyrequestType
                       adc_cmd_autonomyrequest) {
  int x = adc_cmd_autonomyrequest;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 2);
}

Adcmotioncontrol110* Adcmotioncontrol110::set_adc_cmd_longitudinalcontrolmode(
    Adc_motioncontrol1_10::Adc_cmd_longitudinalcontrolmodeType
        adc_cmd_longitudinalcontrolmode) {
  adc_cmd_longitudinalcontrolmode_ = adc_cmd_longitudinalcontrolmode;
  return this;
}

// config detail: {'description': 'Select longitudinal control mode', 'enum':
// {0: 'ADC_CMD_LONGITUDINALCONTROLMODE_NONE', 1:
// 'ADC_CMD_LONGITUDINALCONTROLMODE_RESERVED_VELOCITY_AND_ACCELERATION', 2:
// 'ADC_CMD_LONGITUDINALCONTROLMODE_RESERVED_FORCE', 3:
// 'ADC_CMD_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE'}, 'precision': 1.0,
// 'len': 2, 'name': 'ADC_CMD_LongitudinalControlMode', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 2, 'type': 'enum', 'order':
// 'intel', 'physical_unit': ''}
void Adcmotioncontrol110::set_p_adc_cmd_longitudinalcontrolmode(
    uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_longitudinalcontrolmodeType
                       adc_cmd_longitudinalcontrolmode) {
  int x = adc_cmd_longitudinalcontrolmode;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 2, 2);
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
