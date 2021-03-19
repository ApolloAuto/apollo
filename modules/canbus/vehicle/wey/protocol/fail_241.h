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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace wey {

class Fail241 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Fail241();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Engine Fail status', 'enum': {0:
  // 'ENGFAIL_NO_FAIL', 1: 'ENGFAIL_FAIL'}, 'precision': 1.0, 'len': 1,'name':
  // 'EngFail', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Fail_241::EngfailType engfail(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'description': 'ESP fault', 'enum': {0:
  // 'ESPFAIL_NO_FAILURE', 1: 'ESPFAIL_FAILURE'}, 'precision': 1.0, 'len': 1,
  // 'name': 'ESPFail', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Fail_241::EspfailType espfail(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'description': 'error indication of EPB system', 'enum':
  // {0: 'EPBFAIL_UNDEFINED', 1: 'EPBFAIL_NO_ERROR', 2: 'EPBFAIL_ERROR', 3:
  // 'EPBFAIL_DIAGNOSIS'}, 'precision': 1.0, 'len': 2, 'name': 'EPBFail',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
  // 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Fail_241::EpbfailType epbfail(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'description': 'Driver display failure messages', 'enum':
  // {0: 'SHIFTFAIL_NO_FAIL', 1: 'SHIFTFAIL_TRANSMISSION_MALFUNCTION',
  // 2: 'SHIFTFAIL_TRANSMISSION_P_ENGAGEMENT_FAULT',
  // 3: 'SHIFTFAIL_TRANSMISSION_P_DISENGAGEMENT_FAULT', 4:'SHIFTFAIL_RESERVED',
  // 15: 'SHIFTFAIL_TRANSMISSION_LIMIT_FUNCTION'}, 'precision': 1.0, 'len': 4,
  // 'name': 'ShiftFail', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|15]', 'bit': 31, 'type': 'enum', 'order':'motorola',
  // 'physical_unit': ''}
  Fail_241::ShiftfailType shiftfail(const std::uint8_t* bytes,
                                    const int32_t length) const;

  // config detail: {'description': 'Electrical steering fail status', 'enum':
  // {0: 'EPSFAIL_NO_FAULT', 1: 'EPSFAIL_FAULT'}, 'precision': 1.0, 'len': 1,
  // 'name': 'EPSFail', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Fail_241::EpsfailType epsfail(const std::uint8_t* bytes,
                                const int32_t length) const;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
