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

#include "modules/canbus/vehicle/minibus/protocol/controller_parking_18ff8ca9.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

const int32_t Controllerparking18ff8ca9::ID = 0x38ff8ca9;

// public
Controllerparking18ff8ca9::Controllerparking18ff8ca9() { Reset(); }

uint32_t Controllerparking18ff8ca9::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controllerparking18ff8ca9::UpdateData(uint8_t* data) {
  set_p_cp_epb_enable(data, cp_epb_enable_);
  set_p_cp_park_active(data, cp_park_active_);
}

void Controllerparking18ff8ca9::Reset() {
  // TODO(All) :  you should check this manually
  cp_epb_enable_ = Controller_parking_18ff8ca9::CP_EPB_ENABLE_NO_REQUEST;
  cp_park_active_ = Controller_parking_18ff8ca9::CP_PARK_ACTIVE_NO_ACTIVE;
}

Controllerparking18ff8ca9* Controllerparking18ff8ca9::set_cp_epb_enable(
    Controller_parking_18ff8ca9::Cp_epb_enableType cp_epb_enable) {
  cp_epb_enable_ = cp_epb_enable;
  return this;
}

// config detail: {'bit': 4, 'enum': {0: 'CP_EPB_ENABLE_NO_REQUEST', 1:
// 'CP_EPB_ENABLE_EPB_RELEASE', 2: 'CP_EPB_ENABLE_EPB_TRIGGER'},
// 'is_signed_var': False, 'len': 4, 'name': 'CP_EPB_Enable', 'offset': 0.0,
// 'order': 'intel', 'physical_range': '[0|2]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
void Controllerparking18ff8ca9::set_p_cp_epb_enable(
    uint8_t* data,
    Controller_parking_18ff8ca9::Cp_epb_enableType cp_epb_enable) {
  int x = cp_epb_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Controllerparking18ff8ca9* Controllerparking18ff8ca9::set_cp_park_active(
    Controller_parking_18ff8ca9::Cp_park_activeType cp_park_active) {
  cp_park_active_ = cp_park_active;
  return this;
}

// config detail: {'bit': 0, 'enum': {0: 'CP_PARK_ACTIVE_NO_ACTIVE', 1:
// 'CP_PARK_ACTIVE_ACTIVE'}, 'is_signed_var': False, 'len': 4, 'name':
// 'CP_Park_ACTIVE', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Controllerparking18ff8ca9::set_p_cp_park_active(
    uint8_t* data,
    Controller_parking_18ff8ca9::Cp_park_activeType cp_park_active) {
  int x = cp_park_active;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 4);
}

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
