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

class Llcdiagfault620 : public ::apollo::drivers::canbus::ProtocolData<
                            ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Llcdiagfault620();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;
  FRIEND_TEST(Diag_fault_620_test, General);

 private:
  // config detail: {'description': 'Counts the number of times that the driver
  // has disengaged autonomy by applying the brakes since system reset..',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'LLC_DisengageCounter_Brake', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_disengagecounter_brake(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'description': 'Counts the number of times that the driver
  // has disengaged autonomy by moving the steering wheel since system reset.',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'LLC_DisengageCounter_Steer', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_disengagecounter_steer(const std::uint8_t* bytes,
                                 const int32_t length) const;

  // config detail: {'description': 'Counts the number of times that the driver
  // has disengaged autonomy by applying throttle since system reset.',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'LLC_DisengageCounter_Throttle', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_disengagecounter_throttle(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'description': 'Counts the number of faults that have
  // occurred since system reset.', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'name': 'LLC_FBK_FaultCounter', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_fbk_faultcounter(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'description': 'Counts the number of times that the driver
  // has disengaged autonomy by applying the brakes since system reset..',
  // 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name':
  // 'LLC_DisengageCounter_Button', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int llc_disengagecounter_button(const std::uint8_t* bytes,
                                  const int32_t length) const;

  // config detail: {'description': 'Firmware version', 'offset': 2017.0,
  // 'precision': 1.0, 'len': 7, 'name': 'LLC_FBK_Version_Year',
  // 'is_signed_var': False, 'physical_range': '[2017|2144]', 'bit': 40, 'type':
  // 'int', 'order': 'intel', 'physical_unit': 'g'}
  int llc_fbk_version_year(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'description': 'Firmware version', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'name': 'LLC_FBK_Version_Month',
  // 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 47, 'type':
  // 'int', 'order': 'intel', 'physical_unit': 'Month'}
  int llc_fbk_version_month(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'description': 'Firmware version', 'offset': 0.0,
  // 'precision': 1.0, 'len': 5, 'name': 'LLC_FBK_Version_Day', 'is_signed_var':
  // False, 'physical_range': '[0|31]', 'bit': 51, 'type': 'int', 'order':
  // 'intel', 'physical_unit': 'Day'}
  int llc_fbk_version_day(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'description': 'Firmware version', 'offset': 0.0,
  // 'precision': 1.0, 'len': 5, 'name': 'LLC_FBK_Version_Hour',
  // 'is_signed_var': False, 'physical_range': '[0|31]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': 'Hour'}
  int llc_fbk_version_hour(const std::uint8_t* bytes,
                           const int32_t length) const;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
