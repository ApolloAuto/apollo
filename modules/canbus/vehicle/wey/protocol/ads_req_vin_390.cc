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

#include "modules/canbus/vehicle/wey/protocol/ads_req_vin_390.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

const int32_t Adsreqvin390::ID = 0x390;

// public
Adsreqvin390::Adsreqvin390() { Reset(); }

uint32_t Adsreqvin390::GetPeriod() const {
  // TODO(ChaoMa) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adsreqvin390::UpdateData(uint8_t* data) {
  set_p_req_vin_signal(data, req_vin_signal_);
}

void Adsreqvin390::Reset() {
  // TODO(ChaoMa) :you should check this manually
  req_vin_signal_ = Ads_req_vin_390::REQ_VIN_SIGNAL_NO_REQUEST;
}

Adsreqvin390* Adsreqvin390::set_req_vin_signal(
    Ads_req_vin_390::Req_vin_signalType req_vin_signal) {
  req_vin_signal_ = req_vin_signal;
  return this;
}

// config detail: {'name': 'Req_VIN_Signal', 'enum': {
// 0: 'REQ_VIN_SIGNAL_NO_REQUEST', 1: 'REQ_VIN_SIGNAL_REQUEST'},
// 'precision': 1.0, 'len': 8, 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 7,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adsreqvin390::set_p_req_vin_signal(
    uint8_t* data, Ads_req_vin_390::Req_vin_signalType req_vin_signal) {
  int x = req_vin_signal;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
