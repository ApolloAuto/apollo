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

#include "modules/canbus_vehicle/wey/proto/wey.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace wey {

class Adsreqvin390 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::Wey> {
 public:
  static const int32_t ID;

  Adsreqvin390();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'Req_VIN_Signal', 'enum': {0:
  // 'REQ_VIN_SIGNAL_NO_REQUEST', 1: 'REQ_VIN_SIGNAL_REQUEST'}, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|255]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Adsreqvin390* set_req_vin_signal(
      Ads_req_vin_390::Req_vin_signalType req_vin_signal);

 private:
  // config detail: {'name': 'Req_VIN_Signal', 'enum': {0:
  // 'REQ_VIN_SIGNAL_NO_REQUEST', 1: 'REQ_VIN_SIGNAL_REQUEST'}, 'precision':
  // 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|255]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_req_vin_signal(uint8_t* data,
                            Ads_req_vin_390::Req_vin_signalType req_vin_signal);

 private:
  Ads_req_vin_390::Req_vin_signalType req_vin_signal_;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
