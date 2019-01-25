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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

#include "gtest/gtest_prod.h"

namespace apollo {
namespace canbus {
namespace transit {

class Adcmotioncontrol110 : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adcmotioncontrol110();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Setpoint for steering wheel angle. Positive
  // for CW', 'offset': 0.0, 'precision': -0.05, 'len': 16, 'name':
  // 'ADC_CMD_SteerWheelAngle', 'is_signed_var': True, 'physical_range':
  // '[-1638.4|1638.35]', 'bit': 27, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'deg'}
  Adcmotioncontrol110* set_adc_cmd_steerwheelangle(
      double adc_cmd_steerwheelangle);

  // config detail: {'description': 'Select steering control mode', 'enum': {0:
  // 'ADC_CMD_STEERINGCONTROLMODE_NONE', 1: 'ADC_CMD_STEERINGCONTROLMODE_ANGLE',
  // 2: 'ADC_CMD_STEERINGCONTROLMODE_RESERVED_CURVATURE', 3:
  // 'ADC_CMD_STEERINGCONTROLMODE_RESERVED'}, 'precision': 1.0, 'len': 2,
  // 'name': 'ADC_CMD_SteeringControlMode', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Adcmotioncontrol110* set_adc_cmd_steeringcontrolmode(
      Adc_motioncontrol1_10::Adc_cmd_steeringcontrolmodeType
          adc_cmd_steeringcontrolmode);

  // config detail: {'description': '(Reserved) Control parking brake',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_ParkingBrake',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 53, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  Adcmotioncontrol110* set_adc_cmd_parkingbrake(bool adc_cmd_parkingbrake);

  // config detail: {'description': 'Transmission control - only used in direct
  // longitudinal control', 'enum': {0: 'ADC_CMD_GEAR_P_PARK', 1:
  // 'ADC_CMD_GEAR_D_DRIVE', 2: 'ADC_CMD_GEAR_N_NEUTRAL', 3:
  // 'ADC_CMD_GEAR_R_REVERSE'}, 'precision': 1.0, 'len': 3, 'name':
  // 'ADC_CMD_Gear', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|7]', 'bit': 50, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Adcmotioncontrol110* set_adc_cmd_gear(
      Adc_motioncontrol1_10::Adc_cmd_gearType adc_cmd_gear);

  // config detail: {'description': 'Motion Control 1 checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'ADC_MotionControl1_Checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  Adcmotioncontrol110* set_adc_motioncontrol1_checksum(
      int adc_motioncontrol1_checksum);

  // config detail: {'description': 'Brake pressure for direct longitudinal
  // control', 'offset': 0.0, 'precision': 0.0556, 'len': 11, 'name':
  // 'ADC_CMD_BrakePercentage', 'is_signed_var': False, 'physical_range':
  // '[0|113.8132]', 'bit': 6, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '%'}
  Adcmotioncontrol110* set_adc_cmd_brakepercentage(
      double adc_cmd_brakepercentage);

  // config detail: {'description': 'Throttle pedal position percentage for
  // direct longitudinal control', 'offset': 0.0, 'precision': 0.1, 'len': 10,
  // 'name': 'ADC_CMD_ThrottlePosition', 'is_signed_var': False,
  // 'physical_range': '[0|100]', 'bit': 17, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '%'}
  Adcmotioncontrol110* set_adc_cmd_throttleposition(
      double adc_cmd_throttleposition);

  // config detail: {'description': 'Motion control 1 Heartbeat counter',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'ADC_MotionControl1_Counter', 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Adcmotioncontrol110* set_adc_motioncontrol1_counter(
      int adc_motioncontrol1_counter);

  // config detail: {'description': 'Request from ADC to LLC for autonomy',
  // 'enum': {0: 'ADC_CMD_AUTONOMYREQUEST_AUTONOMY_NOT_REQUESTED', 1:
  // 'ADC_CMD_AUTONOMYREQUEST_AUTONOMY_REQUESTED', 2:
  // 'ADC_CMD_AUTONOMYREQUEST_RESERVED0', 3:
  // 'ADC_CMD_AUTONOMYREQUEST_RESERVED1'}, 'precision': 1.0, 'len': 2, 'name':
  // 'ADC_CMD_AutonomyRequest', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  Adcmotioncontrol110* set_adc_cmd_autonomyrequest(
      Adc_motioncontrol1_10::Adc_cmd_autonomyrequestType
          adc_cmd_autonomyrequest);

  // config detail: {'description': 'Select longitudinal control mode', 'enum':
  // {0: 'ADC_CMD_LONGITUDINALCONTROLMODE_NONE', 1:
  // 'ADC_CMD_LONGITUDINALCONTROLMODE_RESERVED_VELOCITY_AND_ACCELERATION', 2:
  // 'ADC_CMD_LONGITUDINALCONTROLMODE_RESERVED_FORCE', 3:
  // 'ADC_CMD_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE'}, 'precision': 1.0,
  // 'len': 2, 'name': 'ADC_CMD_LongitudinalControlMode', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 2, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  Adcmotioncontrol110* set_adc_cmd_longitudinalcontrolmode(
      Adc_motioncontrol1_10::Adc_cmd_longitudinalcontrolmodeType
          adc_cmd_longitudinalcontrolmode);

  FRIEND_TEST(adc_motioncontrol1_10Test, part1);
  FRIEND_TEST(adc_motioncontrol1_10Test, part2);
  FRIEND_TEST(adc_motioncontrol1_10Test, part3);
  FRIEND_TEST(adc_motioncontrol1_10Test, part4);

 private:
  // config detail: {'description': 'Setpoint for steering wheel angle. Positive
  // for CW', 'offset': 0.0, 'precision': -0.05, 'len': 16, 'name':
  // 'ADC_CMD_SteerWheelAngle', 'is_signed_var': True, 'physical_range':
  // '[-1638.4|1638.35]', 'bit': 27, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'deg'}
  void set_p_adc_cmd_steerwheelangle(uint8_t* data,
                                     double adc_cmd_steerwheelangle);

  // config detail: {'description': 'Select steering control mode', 'enum': {0:
  // 'ADC_CMD_STEERINGCONTROLMODE_NONE', 1: 'ADC_CMD_STEERINGCONTROLMODE_ANGLE',
  // 2: 'ADC_CMD_STEERINGCONTROLMODE_RESERVED_CURVATURE', 3:
  // 'ADC_CMD_STEERINGCONTROLMODE_RESERVED'}, 'precision': 1.0, 'len': 2,
  // 'name': 'ADC_CMD_SteeringControlMode', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_adc_cmd_steeringcontrolmode(
      uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_steeringcontrolmodeType
                         adc_cmd_steeringcontrolmode);

  // config detail: {'description': '(Reserved) Control parking brake',
  // 'offset': 0.0, 'precision': 1.0, 'len': 1, 'name': 'ADC_CMD_ParkingBrake',
  // 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 53, 'type':
  // 'bool', 'order': 'intel', 'physical_unit': 'T/F'}
  void set_p_adc_cmd_parkingbrake(uint8_t* data, bool adc_cmd_parkingbrake);

  // config detail: {'description': 'Transmission control - only used in direct
  // longitudinal control', 'enum': {0: 'ADC_CMD_GEAR_P_PARK', 1:
  // 'ADC_CMD_GEAR_D_DRIVE', 2: 'ADC_CMD_GEAR_N_NEUTRAL', 3:
  // 'ADC_CMD_GEAR_R_REVERSE'}, 'precision': 1.0, 'len': 3, 'name':
  // 'ADC_CMD_Gear', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|7]', 'bit': 50, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_cmd_gear(uint8_t* data,
                          Adc_motioncontrol1_10::Adc_cmd_gearType adc_cmd_gear);

  // config detail: {'description': 'Motion Control 1 checksum', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'name': 'ADC_MotionControl1_Checksum',
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_motioncontrol1_checksum(uint8_t* data,
                                         int adc_motioncontrol1_checksum);

  // config detail: {'description': 'Brake pressure for direct longitudinal
  // control', 'offset': 0.0, 'precision': 0.0556, 'len': 11, 'name':
  // 'ADC_CMD_BrakePercentage', 'is_signed_var': False, 'physical_range':
  // '[0|113.8132]', 'bit': 6, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '%'}
  void set_p_adc_cmd_brakepercentage(uint8_t* data,
                                     double adc_cmd_brakepercentage);

  // config detail: {'description': 'Throttle pedal position percentage for
  // direct longitudinal control', 'offset': 0.0, 'precision': 0.1, 'len': 10,
  // 'name': 'ADC_CMD_ThrottlePosition', 'is_signed_var': False,
  // 'physical_range': '[0|100]', 'bit': 17, 'type': 'double', 'order': 'intel',
  // 'physical_unit': '%'}
  void set_p_adc_cmd_throttleposition(uint8_t* data,
                                      double adc_cmd_throttleposition);

  // config detail: {'description': 'Motion control 1 Heartbeat counter',
  // 'offset': 0.0, 'precision': 1.0, 'len': 2, 'name':
  // 'ADC_MotionControl1_Counter', 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 54, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_motioncontrol1_counter(uint8_t* data,
                                        int adc_motioncontrol1_counter);

  // config detail: {'description': 'Request from ADC to LLC for autonomy',
  // 'enum': {0: 'ADC_CMD_AUTONOMYREQUEST_AUTONOMY_NOT_REQUESTED', 1:
  // 'ADC_CMD_AUTONOMYREQUEST_AUTONOMY_REQUESTED', 2:
  // 'ADC_CMD_AUTONOMYREQUEST_RESERVED0', 3:
  // 'ADC_CMD_AUTONOMYREQUEST_RESERVED1'}, 'precision': 1.0, 'len': 2, 'name':
  // 'ADC_CMD_AutonomyRequest', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 0, 'type': 'enum', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_adc_cmd_autonomyrequest(
      uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_autonomyrequestType
                         adc_cmd_autonomyrequest);

  // config detail: {'description': 'Select longitudinal control mode', 'enum':
  // {0: 'ADC_CMD_LONGITUDINALCONTROLMODE_NONE', 1:
  // 'ADC_CMD_LONGITUDINALCONTROLMODE_RESERVED_VELOCITY_AND_ACCELERATION', 2:
  // 'ADC_CMD_LONGITUDINALCONTROLMODE_RESERVED_FORCE', 3:
  // 'ADC_CMD_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE'}, 'precision': 1.0,
  // 'len': 2, 'name': 'ADC_CMD_LongitudinalControlMode', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 2, 'type': 'enum',
  // 'order': 'intel', 'physical_unit': ''}
  void set_p_adc_cmd_longitudinalcontrolmode(
      uint8_t* data, Adc_motioncontrol1_10::Adc_cmd_longitudinalcontrolmodeType
                         adc_cmd_longitudinalcontrolmode);

 private:
  double adc_cmd_steerwheelangle_;
  Adc_motioncontrol1_10::Adc_cmd_steeringcontrolmodeType
      adc_cmd_steeringcontrolmode_;
  bool adc_cmd_parkingbrake_;
  Adc_motioncontrol1_10::Adc_cmd_gearType adc_cmd_gear_;
  int adc_motioncontrol1_checksum_;
  double adc_cmd_brakepercentage_;
  double adc_cmd_throttleposition_;
  int adc_motioncontrol1_counter_;
  Adc_motioncontrol1_10::Adc_cmd_autonomyrequestType adc_cmd_autonomyrequest_;
  Adc_motioncontrol1_10::Adc_cmd_longitudinalcontrolmodeType
      adc_cmd_longitudinalcontrolmode_;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
