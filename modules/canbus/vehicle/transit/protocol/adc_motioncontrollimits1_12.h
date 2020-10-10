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

#include "gtest/gtest.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transit {

class Adcmotioncontrollimits112
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adcmotioncontrollimits112();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  FRIEND_TEST(Motioncontrollimits1_12_test, General);

  // config detail: {'description': 'Set limit for throttle position', 'offset':
  // 0.0, 'precision': 0.5, 'len': 8, 'name': 'ADC_CMD_ThrottleCommandLimit',
  // 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 24, 'type':
  // 'double', 'order': 'intel', 'physical_unit': '%'}
  Adcmotioncontrollimits112* set_adc_cmd_throttlecommandlimit(
      double adc_cmd_throttlecommandlimit);

  // config detail: {'description': 'Set steering rate', 'offset': 0.0,
  // 'precision': 0.05, 'len': 16, 'name': 'ADC_CMD_SteeringRate',
  // 'is_signed_var': False, 'physical_range': '[0|3276.75]', 'bit': 0, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'deg/s'}
  Adcmotioncontrollimits112* set_adc_cmd_steeringrate(
      double adc_cmd_steeringrate);

  // config detail: {'description': 'Set limit for steering wheel angle. Applies
  // in both positive and negative', 'offset': 0.0, 'precision': 5.0, 'len': 8,
  // 'name': 'ADC_CMD_SteerWheelAngleLimit', 'is_signed_var': False,
  // 'physical_range': '[0|1275]', 'bit': 16, 'type': 'double', 'order':
  // 'intel', 'physical_unit': 'deg'}
  Adcmotioncontrollimits112* set_adc_cmd_steerwheelanglelimit(
      double adc_cmd_steerwheelanglelimit);

 private:
  // config detail: {'description': 'Set limit for throttle position', 'offset':
  // 0.0, 'precision': 0.5, 'len': 8, 'name': 'ADC_CMD_ThrottleCommandLimit',
  // 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 24, 'type':
  // 'double', 'order': 'intel', 'physical_unit': '%'}
  void set_p_adc_cmd_throttlecommandlimit(uint8_t* data,
                                          double adc_cmd_throttlecommandlimit);

  // config detail: {'description': 'Set steering rate', 'offset': 0.0,
  // 'precision': 0.05, 'len': 16, 'name': 'ADC_CMD_SteeringRate',
  // 'is_signed_var': False, 'physical_range': '[0|3276.75]', 'bit': 0, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'deg/s'}
  void set_p_adc_cmd_steeringrate(uint8_t* data, double adc_cmd_steeringrate);

  // config detail: {'description': 'Set limit for steering wheel angle. Applies
  // in both positive and negative', 'offset': 0.0, 'precision': 5.0, 'len': 8,
  // 'name': 'ADC_CMD_SteerWheelAngleLimit', 'is_signed_var': False,
  // 'physical_range': '[0|1275]', 'bit': 16, 'type': 'double', 'order':
  // 'intel', 'physical_unit': 'deg'}
  void set_p_adc_cmd_steerwheelanglelimit(uint8_t* data,
                                          double adc_cmd_steerwheelanglelimit);

 private:
  double adc_cmd_throttlecommandlimit_;
  double adc_cmd_steeringrate_;
  double adc_cmd_steerwheelanglelimit_;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
