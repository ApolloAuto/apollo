/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

class License7e : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::ChassisDetail> {
 private:
  mutable std::string vin_part0_;
  mutable std::string vin_part1_;
  mutable std::string vin_part2_;
  mutable bool vin_part0_flag_;
  mutable bool vin_part1_flag_;
  mutable bool vin_part2_flag_;
  mutable bool parse_success_;

 public:
  static const int32_t ID;
  License7e();
  virtual void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis_detail) const;
  // config detail: {'name': 'mux', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mux(const std::uint8_t* bytes, int length) const;

  bool is_ready(const std::uint8_t* bytes, int length) const;

  bool is_trial(const std::uint8_t* bytes, int length) const;

  bool is_expired(const std::uint8_t* bytes, int length) const;

  bool is_feat_base_enabled(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date0', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date0(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date6', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date6(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'mac0', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mac0(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin00', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin00(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin06', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin06(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin12', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin12(const std::uint8_t* bytes, int length) const;

  bool is_feat_base_trial(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date1', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date1(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date7', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date7(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'mac1', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mac1(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin01', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin01(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin07', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin07(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin13', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin13(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date2', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date2(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date8', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date8(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'mac2', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mac2(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin02', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin02(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin08', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin08(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin14', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin14(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'feat_base_trials_used', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 32, 'type': 'int', 'order': 'intel',
  // 'physical_unit': '""'}
  int feat_base_trials_used(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date3', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date3(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date9', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date9(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'mac3', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mac3(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin03', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin03(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin09', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin09(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin15', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin15(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date4', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date4(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'mac4', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mac4(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin04', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin04(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin10', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin10(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin16', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin16(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'feat_base_trials_remaining', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'f_type': 'value', 'is_signed_var': False,
  // 'physical_range': '[0|0]', 'bit': 48, 'type': 'int', 'order': 'intel',
  // 'physical_unit': '""'}
  int feat_base_trials_remaining(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'date5', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int date5(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'mac5', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  int mac5(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin05', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin05(const std::uint8_t* bytes, int length) const;

  // config detail: {'name': 'vin11', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
  // 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
  std::string vin11(const std::uint8_t* bytes, int length) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
