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

#include "modules/canbus/vehicle/wey/protocol/ads1_111.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

const int32_t Ads1111::ID = 0x111;

// public
Ads1111::Ads1111() { Reset(); }

uint32_t Ads1111::GetPeriod() const {
  // TODO(ChaoMa) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Ads1111::UpdateData(uint8_t* data) {
  set_p_ads_dectostop(data, ads_dectostop_);
  set_p_ads_mode(data, ads_mode_);
  set_p_ads_taracce(data, ads_taracce_);
  set_p_ads_driveoff_req(data, ads_driveoff_req_);
  set_p_ads_aeb_taracce(data, ads_aeb_taracce_);
  set_p_ads_aeb_tgtdecel_req(data, ads_aeb_tgtdecel_req_);
}

void Ads1111::Reset() {
  // TODO(ChaoMa) :you should check this manually
  ads_dectostop_ = Ads1_111::ADS_DECTOSTOP_NO_DEMAND;
  ads_mode_ = Ads1_111::ADS_MODE_OFF_MODE;
  ads_taracce_ = 0.0;
  ads_driveoff_req_ = Ads1_111::ADS_DRIVEOFF_REQ_NO_DEMAND;
  ads_aeb_taracce_ = 0.0;
  ads_aeb_tgtdecel_req_ = Ads1_111::ADS_AEB_TGTDECEL_REQ_NO_DEMAND;
}

Ads1111* Ads1111::set_ads_dectostop(Ads1_111::Ads_dectostopType ads_dectostop) {
  ads_dectostop_ = ads_dectostop;
  return this;
}

// config detail: {'description': 'ADS request ESP/VLC to decelerate until
// standstill.', 'enum': {0: 'ADS_DECTOSTOP_NO_DEMAND', 1:
// 'ADS_DECTOSTOP_DEMAND'}, 'precision': 1.0, 'len': 1, 'name': 'ADS_DecToStop',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ads1111::set_p_ads_dectostop(uint8_t* data,
                                  Ads1_111::Ads_dectostopType ads_dectostop) {
  int x = ads_dectostop;

  Byte to_set(data + 2);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Ads1111* Ads1111::set_ads_mode(Ads1_111::Ads_modeType ads_mode) {
  ads_mode_ = ads_mode;
  return this;
}

// config detail: {'description': 'The status of the ADS control unit.
// The ADS mode should be contained in every message sent by ADS', 'enum':
// {0: 'ADS_MODE_OFF_MODE', 3: 'ADS_MODE_ACTIVE_MODE'}, 'precision': 1.0,
// 'len': 5, 'name': 'ADS_Mode', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|31]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads1111::set_p_ads_mode(uint8_t* data, Ads1_111::Ads_modeType ads_mode) {
  int x = ads_mode;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 3, 5);
}

Ads1111* Ads1111::set_ads_taracce(double ads_taracce) {
  ads_taracce_ = ads_taracce;
  return this;
}

// config detail: {'description': 'ADS target acceleration for transmission',
// 'offset': -7.0, 'precision': 0.05, 'len': 8, 'name': 'ADS_TarAcce',
// 'is_signed_var': False, 'physical_range': '[-7|5.75]', 'bit': 15, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s2'}
void Ads1111::set_p_ads_taracce(uint8_t* data, double ads_taracce) {
  ads_taracce = ProtocolData::BoundedValue(-7.0, 5.75, ads_taracce);
  int x = static_cast<int>((ads_taracce - -7.000000) / 0.050000);

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 8);
}

Ads1111* Ads1111::set_ads_driveoff_req(
    Ads1_111::Ads_driveoff_reqType ads_driveoff_req) {
  ads_driveoff_req_ = ads_driveoff_req;
  return this;
}

// config detail: {'description': 'ACC request ESP drive off', 'enum': {0:
// 'ADS_DRIVEOFF_REQ_NO_DEMAND', 1: 'ADS_DRIVEOFF_REQ_DEMAND'}, 'precision':1.0,
// 'len': 1, 'name': 'ADS_Driveoff_Req', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads1111::set_p_ads_driveoff_req(
    uint8_t* data, Ads1_111::Ads_driveoff_reqType ads_driveoff_req) {
  int x = ads_driveoff_req;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Ads1111* Ads1111::set_ads_aeb_taracce(double ads_aeb_taracce) {
  ads_aeb_taracce_ = ads_aeb_taracce;
  return this;
}

// config detail: {'description': 'target deceleration value from AEB',
// 'offset': -16.0, 'precision': 0.000488, 'len': 16, 'name': 'ADS_AEB_TarAcce',
// 'is_signed_var': False, 'physical_range': '[-16|16]', 'bit': 39, 'type':
// 'double', 'order': 'motorola', 'physical_unit': 'm/s2'}
void Ads1111::set_p_ads_aeb_taracce(uint8_t* data, double ads_aeb_taracce) {
  ads_aeb_taracce = ProtocolData::BoundedValue(-16.0, 16.0, ads_aeb_taracce);
  int x = static_cast<int>((ads_aeb_taracce - -16.000000) / 0.000488);
  uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 5);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 8);
}

Ads1111* Ads1111::set_ads_aeb_tgtdecel_req(
    Ads1_111::Ads_aeb_tgtdecel_reqType ads_aeb_tgtdecel_req) {
  ads_aeb_tgtdecel_req_ = ads_aeb_tgtdecel_req;
  return this;
}

// config detail: {'description': 'Request of the AEB deceleration control.',
// 'enum': {0: 'ADS_AEB_TGTDECEL_REQ_NO_DEMAND', 1:
// 'ADS_AEB_TGTDECEL_REQ_DEMAND'}, 'precision': 1.0, 'len': 1,
// 'name': 'ADS_AEB_TgtDecel_Req', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order':'motorola',
// 'physical_unit': ''}
void Ads1111::set_p_ads_aeb_tgtdecel_req(
    uint8_t* data, Ads1_111::Ads_aeb_tgtdecel_reqType ads_aeb_tgtdecel_req) {
  int x = ads_aeb_tgtdecel_req;

  Byte to_set(data + 3);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
