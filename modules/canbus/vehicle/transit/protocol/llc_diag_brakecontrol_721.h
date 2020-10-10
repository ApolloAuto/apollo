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

class Llcdiagbrakecontrol721 : public ::apollo::drivers::canbus::ProtocolData<
                                   ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Llcdiagbrakecontrol721();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Brake control loop P contribution',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_DBG_BrakePidContribution_P', 'is_signed_var': True, 'physical_range':
  // '[-51.2|51.1]', 'bit': 34, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  Llcdiagbrakecontrol721* set_llc_dbg_brakepidcontribution_p(
      double llc_dbg_brakepidcontribution_p);

  // config detail: {'description': 'Brake control loop I contribution',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_DBG_BrakePidContribution_I', 'is_signed_var': True, 'physical_range':
  // '[-51.2|51.1]', 'bit': 44, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  Llcdiagbrakecontrol721* set_llc_dbg_brakepidcontribution_i(
      double llc_dbg_brakepidcontribution_i);

  // config detail: {'description': 'Brake control loop D contribution',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_DBG_BrakePidContribution_D', 'is_signed_var': True, 'physical_range':
  // '[-51.2|51.1]', 'bit': 54, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  Llcdiagbrakecontrol721* set_llc_dbg_brakepidcontribution_d(
      double llc_dbg_brakepidcontribution_d);

  // config detail: {'description': 'Brake control loop output', 'offset': 0.0,
  // 'precision': 0.1, 'len': 10, 'name': 'LLC_DBG_BrakePID_Output',
  // 'is_signed_var': True, 'physical_range': '[-51.2|51.1]', 'bit': 12, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'mrev'}
  Llcdiagbrakecontrol721* set_llc_dbg_brakepid_output(
      double llc_dbg_brakepid_output);

  // config detail: {'description': 'Brake control loop error', 'offset': 0.0,
  // 'precision': 1.0, 'len': 12, 'name': 'LLC_DBG_BrakePID_Error',
  // 'is_signed_var': True, 'physical_range': '[-2048|2047]', 'bit': 0, 'type':
  // 'int', 'order': 'intel', 'physical_unit': 'psi'}
  Llcdiagbrakecontrol721* set_llc_dbg_brakepid_error(
      int llc_dbg_brakepid_error);

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 0.5, 'len': 12, 'name':
  // 'LLC_DBG_BrakeFeedforward', 'is_signed_var': True, 'physical_range':
  // '[-1024|1023.5]', 'bit': 22, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  Llcdiagbrakecontrol721* set_llc_dbg_brakefeedforward(
      double llc_dbg_brakefeedforward);

  FRIEND_TEST(llc_diag_brakecontrol_721Test, part1);
  FRIEND_TEST(llc_diag_brakecontrol_721Test, part2);

 private:
  // config detail: {'description': 'Brake control loop P contribution',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_DBG_BrakePidContribution_P', 'is_signed_var': True, 'physical_range':
  // '[-51.2|51.1]', 'bit': 34, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  void set_p_llc_dbg_brakepidcontribution_p(
      uint8_t* data, double llc_dbg_brakepidcontribution_p);

  // config detail: {'description': 'Brake control loop I contribution',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_DBG_BrakePidContribution_I', 'is_signed_var': True, 'physical_range':
  // '[-51.2|51.1]', 'bit': 44, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  void set_p_llc_dbg_brakepidcontribution_i(
      uint8_t* data, double llc_dbg_brakepidcontribution_i);

  // config detail: {'description': 'Brake control loop D contribution',
  // 'offset': 0.0, 'precision': 0.1, 'len': 10, 'name':
  // 'LLC_DBG_BrakePidContribution_D', 'is_signed_var': True, 'physical_range':
  // '[-51.2|51.1]', 'bit': 54, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  void set_p_llc_dbg_brakepidcontribution_d(
      uint8_t* data, double llc_dbg_brakepidcontribution_d);

  // config detail: {'description': 'Brake control loop output', 'offset': 0.0,
  // 'precision': 0.1, 'len': 10, 'name': 'LLC_DBG_BrakePID_Output',
  // 'is_signed_var': True, 'physical_range': '[-51.2|51.1]', 'bit': 12, 'type':
  // 'double', 'order': 'intel', 'physical_unit': 'mrev'}
  void set_p_llc_dbg_brakepid_output(uint8_t* data,
                                     double llc_dbg_brakepid_output);

  // config detail: {'description': 'Brake control loop error', 'offset': 0.0,
  // 'precision': 1.0, 'len': 12, 'name': 'LLC_DBG_BrakePID_Error',
  // 'is_signed_var': True, 'physical_range': '[-2048|2047]', 'bit': 0, 'type':
  // 'int', 'order': 'intel', 'physical_unit': 'psi'}
  void set_p_llc_dbg_brakepid_error(uint8_t* data, int llc_dbg_brakepid_error);

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 0.5, 'len': 12, 'name':
  // 'LLC_DBG_BrakeFeedforward', 'is_signed_var': True, 'physical_range':
  // '[-1024|1023.5]', 'bit': 22, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'mrev'}
  void set_p_llc_dbg_brakefeedforward(uint8_t* data,
                                      double llc_dbg_brakefeedforward);

 private:
  double llc_dbg_brakepidcontribution_p_;
  double llc_dbg_brakepidcontribution_i_;
  double llc_dbg_brakepidcontribution_d_;
  double llc_dbg_brakepid_output_;
  int llc_dbg_brakepid_error_;
  double llc_dbg_brakefeedforward_;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
