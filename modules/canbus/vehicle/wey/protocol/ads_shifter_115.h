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

class Adsshifter115 : public ::apollo::drivers::canbus::ProtocolData<
                          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adsshifter115();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'ADS Shift Mode', 'enum': {0:
  // 'ADS_SHIFTMODE_INVALID', 1: 'ADS_SHIFTMODE_VALID'}, 'precision': 1.0,
  // 'len': 4, 'name': 'ADS_ShiftMode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Adsshifter115* set_ads_shiftmode(
      Ads_shifter_115::Ads_shiftmodeType ads_shiftmode);

  // config detail: {'description': 'ADS Target Gear', 'enum': {0:
  // 'ADS_TARGETGEAR_N', 1: 'ADS_TARGETGEAR_R', 2: 'ADS_TARGETGEAR_P', 3:
  // 'ADS_TARGETGEAR_D'}, 'precision': 1.0, 'len': 2, 'name': 'ADS_TargetGear',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]',
  // 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adsshifter115* set_ads_targetgear(
      Ads_shifter_115::Ads_targetgearType ads_targetgear);

 private:
  // config detail: {'description': 'ADS Shift Mode', 'enum': {0:
  // 'ADS_SHIFTMODE_INVALID', 1: 'ADS_SHIFTMODE_VALID'}, 'precision': 1.0,
  // 'len': 4, 'name': 'ADS_ShiftMode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_shiftmode(uint8_t* data,
                           Ads_shifter_115::Ads_shiftmodeType ads_shiftmode);

  // config detail: {'description': 'ADS Target Gear', 'enum': {0:
  // 'ADS_TARGETGEAR_N', 1: 'ADS_TARGETGEAR_R', 2: 'ADS_TARGETGEAR_P', 3:
  // 'ADS_TARGETGEAR_D'}, 'precision': 1.0, 'len': 2, 'name': 'ADS_TargetGear',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]',
  // 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ads_targetgear(uint8_t* data,
                            Ads_shifter_115::Ads_targetgearType ads_targetgear);

 private:
  Ads_shifter_115::Ads_shiftmodeType ads_shiftmode_;
  Ads_shifter_115::Ads_targetgearType ads_targetgear_;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
