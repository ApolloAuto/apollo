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

#include "modules/canbus/vehicle/wey/protocol/ads_shifter_115.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

const int32_t Adsshifter115::ID = 0x115;

// public
Adsshifter115::Adsshifter115() { Reset(); }

uint32_t Adsshifter115::GetPeriod() const {
  // TODO(ChaoMa) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adsshifter115::UpdateData(uint8_t* data) {
  set_p_ads_shiftmode(data, ads_shiftmode_);
  set_p_ads_targetgear(data, ads_targetgear_);
}

void Adsshifter115::Reset() {
  // TODO(ChaoMa) :you should check this manually
  ads_shiftmode_ = Ads_shifter_115::ADS_SHIFTMODE_INVALID;
  ads_targetgear_ = Ads_shifter_115::ADS_TARGETGEAR_N;
}

Adsshifter115* Adsshifter115::set_ads_shiftmode(
    Ads_shifter_115::Ads_shiftmodeType ads_shiftmode) {
  ads_shiftmode_ = ads_shiftmode;
  return this;
}

// config detail: {'description': 'ADS Shift Mode', 'enum':
// {0: 'ADS_SHIFTMODE_INVALID', 1: 'ADS_SHIFTMODE_VALID'}, 'precision': 1.0,
// 'len': 4, 'name': 'ADS_ShiftMode', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Adsshifter115::set_p_ads_shiftmode(
    uint8_t* data, Ads_shifter_115::Ads_shiftmodeType ads_shiftmode) {
  int x = ads_shiftmode;

  Byte to_set(data + 3);
  to_set.set_value(static_cast<uint8_t>(x), 4, 4);
}

Adsshifter115* Adsshifter115::set_ads_targetgear(
    Ads_shifter_115::Ads_targetgearType ads_targetgear) {
  ads_targetgear_ = ads_targetgear;
  return this;
}

// config detail: {'description': 'ADS Target Gear', 'enum':
// {0: 'ADS_TARGETGEAR_N', 1: 'ADS_TARGETGEAR_R', 2: 'ADS_TARGETGEAR_P',
// 3: 'ADS_TARGETGEAR_D'}, 'precision': 1.0, 'len': 2, 'name': 'ADS_TargetGear',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 38,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adsshifter115::set_p_ads_targetgear(
    uint8_t* data, Ads_shifter_115::Ads_targetgearType ads_targetgear) {
  int x = ads_targetgear;

  Byte to_set(data + 4);
  to_set.set_value(static_cast<uint8_t>(x), 5, 2);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
