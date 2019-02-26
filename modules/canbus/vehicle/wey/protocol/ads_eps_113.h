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

class Adseps113 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adseps113();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'EPS Mode', 'enum': {0:
  // 'ADS_EPSMODE_DISABLE', 2: 'ADS_EPSMODE_ACTIVE'}, 'precision': 1.0,
  // 'len': 2, 'name': 'ADS_EPSMode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Adseps113* set_ads_epsmode(Ads_eps_113::Ads_epsmodeType ads_epsmode);

  // config detail: {'description': 'Steering Wheel Target Angle', 'offset':
  // -800.0, 'precision': 0.1, 'len': 14, 'name': 'ADS_ReqEPSTargetAngle',
  // 'is_signed_var': False, 'physical_range': '[-800|838.3]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
  Adseps113* set_ads_reqepstargetangle(double ads_reqepstargetangle);

 private:
  // config detail: {'description': 'EPS Mode', 'enum': {0:
  // 'ADS_EPSMODE_DISABLE', 2: 'ADS_EPSMODE_ACTIVE'}, 'precision': 1.0,
  // 'len': 2, 'name': 'ADS_EPSMode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_epsmode(uint8_t* data,
                         Ads_eps_113::Ads_epsmodeType ads_epsmode);

  // config detail: {'description': 'Steering Wheel Target Angle',
  // 'offset': -800.0, 'precision': 0.1, 'len': 14, 'name':
  // 'ADS_ReqEPSTargetAngle', 'is_signed_var': False, 'physical_range':
  // '[-800|838.3]', 'bit': 15, 'type': 'double', 'order': 'motorola',
  // 'physical_unit': 'deg'}
  void set_p_ads_reqepstargetangle(uint8_t* data, double ads_reqepstargetangle);

 private:
  Ads_eps_113::Ads_epsmodeType ads_epsmode_;
  double ads_reqepstargetangle_ = 0.0;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
