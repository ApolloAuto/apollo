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
namespace minibus {

class Controllerparking18ff8ca9
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controllerparking18ff8ca9();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 4, 'enum': {0: 'CP_EPB_ENABLE_NO_REQUEST', 1:
  // 'CP_EPB_ENABLE_EPB_RELEASE', 2: 'CP_EPB_ENABLE_EPB_TRIGGER'},
  // 'is_signed_var': False, 'len': 4, 'name': 'CP_EPB_Enable', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Controllerparking18ff8ca9* set_cp_epb_enable(
      Controller_parking_18ff8ca9::Cp_epb_enableType cp_epb_enable);

  // config detail: {'bit': 0, 'enum': {0: 'CP_PARK_ACTIVE_NO_ACTIVE', 1:
  // 'CP_PARK_ACTIVE_ACTIVE'}, 'is_signed_var': False, 'len': 4, 'name':
  // 'CP_Park_ACTIVE', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Controllerparking18ff8ca9* set_cp_park_active(
      Controller_parking_18ff8ca9::Cp_park_activeType cp_park_active);

 private:
  // config detail: {'bit': 4, 'enum': {0: 'CP_EPB_ENABLE_NO_REQUEST', 1:
  // 'CP_EPB_ENABLE_EPB_RELEASE', 2: 'CP_EPB_ENABLE_EPB_TRIGGER'},
  // 'is_signed_var': False, 'len': 4, 'name': 'CP_EPB_Enable', 'offset': 0.0,
  // 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  void set_p_cp_epb_enable(
      uint8_t* data,
      Controller_parking_18ff8ca9::Cp_epb_enableType cp_epb_enable);

  // config detail: {'bit': 0, 'enum': {0: 'CP_PARK_ACTIVE_NO_ACTIVE', 1:
  // 'CP_PARK_ACTIVE_ACTIVE'}, 'is_signed_var': False, 'len': 4, 'name':
  // 'CP_Park_ACTIVE', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_cp_park_active(
      uint8_t* data,
      Controller_parking_18ff8ca9::Cp_park_activeType cp_park_active);

 private:
  Controller_parking_18ff8ca9::Cp_epb_enableType cp_epb_enable_;
  Controller_parking_18ff8ca9::Cp_park_activeType cp_park_active_;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
