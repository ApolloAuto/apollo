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

#include "modules/canbus/vehicle/wey/protocol/ads_eps_113.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

const int32_t Adseps113::ID = 0x113;

// public
Adseps113::Adseps113() { Reset(); }

uint32_t Adseps113::GetPeriod() const {
  // TODO(ChaoMa) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adseps113::UpdateData(uint8_t* data) {
  set_p_ads_epsmode(data, ads_epsmode_);
  set_p_ads_reqepstargetangle(data, ads_reqepstargetangle_);
}

void Adseps113::Reset() {
  // TODO(ChaoMa) :you should check this manually
  ads_epsmode_ = Ads_eps_113::ADS_EPSMODE_DISABLE;
  ads_reqepstargetangle_ = 0.0;
}

Adseps113* Adseps113::set_ads_epsmode(
    Ads_eps_113::Ads_epsmodeType ads_epsmode) {
  ads_epsmode_ = ads_epsmode;
  return this;
}

// config detail: {'description': 'EPS Mode', 'enum': {0: 'ADS_EPSMODE_DISABLE'
// 2: 'ADS_EPSMODE_ACTIVE'}, 'precision': 1.0, 'len': 2, 'name': 'ADS_EPSMode',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adseps113::set_p_ads_epsmode(uint8_t* data,
                                  Ads_eps_113::Ads_epsmodeType ads_epsmode) {
  int x = ads_epsmode;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 6, 2);
}

Adseps113* Adseps113::set_ads_reqepstargetangle(double ads_reqepstargetangle) {
  ads_reqepstargetangle_ = ads_reqepstargetangle;
  return this;
}

// config detail: {'description': 'Steering Wheel Target Angle', 'offset':
// -800.0, 'precision': 0.1, 'len': 14, 'name': 'ADS_ReqEPSTargetAngle',
// 'is_signed_var': False, 'physical_range': '[-800|838.3]', 'bit': 15,
// 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
void Adseps113::set_p_ads_reqepstargetangle(uint8_t* data,
                                            double ads_reqepstargetangle) {
  ads_reqepstargetangle =
      ProtocolData::BoundedValue(-800.0, 838.3, ads_reqepstargetangle);
  int x = static_cast<int>((ads_reqepstargetangle - -800.000000) / 0.100000);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0x3F);
  Byte to_set0(data + 2);
  to_set0.set_value(t, 2, 6);
  x >>= 6;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
