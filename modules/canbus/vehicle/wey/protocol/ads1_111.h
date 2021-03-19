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

class Ads1111 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Ads1111();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'ADS request ESP/VLC to decelerate until
  // standstill.', 'enum': {0: 'ADS_DECTOSTOP_NO_DEMAND', 1:
  // 'ADS_DECTOSTOP_DEMAND'}, 'precision': 1.0, 'len': 1, 'name':
  // 'ADS_DecToStop', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads1111* set_ads_dectostop(Ads1_111::Ads_dectostopType ads_dectostop);

  // config detail: {'description': 'The status of the ADS control unit.The ADS
  // mode should be contained in every message sent by ADS', 'enum': {0:
  // 'ADS_MODE_OFF_MODE', 3: 'ADS_MODE_ACTIVE_MODE'}, 'precision': 1.0, 'len':
  // 5, 'name': 'ADS_Mode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|31]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads1111* set_ads_mode(Ads1_111::Ads_modeType ads_mode);

  // config detail: {'description': 'ADS target acceleration for transmission',
  // 'offset': -7.0, 'precision': 0.05, 'len': 8, 'name': 'ADS_TarAcce',
  // 'is_signed_var': False, 'physical_range': '[-7|5.75]', 'bit': 15, 'type':
  // 'double', 'order': 'motorola', 'physical_unit': 'm/s2'}
  Ads1111* set_ads_taracce(double ads_taracce);

  // config detail: {'description': 'ACC request ESP drive off', 'enum': {0:
  // 'ADS_DRIVEOFF_REQ_NO_DEMAND', 1: 'ADS_DRIVEOFF_REQ_DEMAND'}, 'precision':
  // 1.0, 'len': 1, 'name': 'ADS_Driveoff_Req', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Ads1111* set_ads_driveoff_req(
      Ads1_111::Ads_driveoff_reqType ads_driveoff_req);

  // config detail: {'description': 'target deceleration value from AEB',
  // 'offset': -16.0, 'precision': 0.000488, 'len': 16, 'name':
  // 'ADS_AEB_TarAcce', 'is_signed_var': False, 'physical_range': '[-16|16]',
  // 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s2'}
  Ads1111* set_ads_aeb_taracce(double ads_aeb_taracce);

  // config detail: {'description': 'Request of the AEB deceleration control.',
  // 'enum': {0: 'ADS_AEB_TGTDECEL_REQ_NO_DEMAND', 1:
  // 'ADS_AEB_TGTDECEL_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'name':
  // 'ADS_AEB_TgtDecel_Req', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads1111* set_ads_aeb_tgtdecel_req(
      Ads1_111::Ads_aeb_tgtdecel_reqType ads_aeb_tgtdecel_req);

 private:
  // config detail: {'description': 'ADS request ESP/VLC to decelerate until
  // standstill.', 'enum': {0: 'ADS_DECTOSTOP_NO_DEMAND', 1:
  // 'ADS_DECTOSTOP_DEMAND'}, 'precision': 1.0, 'len': 1, 'name':
  // 'ADS_DecToStop', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
  // '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_dectostop(uint8_t* data,
                           Ads1_111::Ads_dectostopType ads_dectostop);

  // config detail: {'description': 'The status of the ADS control unit.The
  // ADS mode should be contained in every message sent by ADS', 'enum': {0:
  // 'ADS_MODE_OFF_MODE', 3: 'ADS_MODE_ACTIVE_MODE'}, 'precision': 1.0,
  // 'len': 5, 'name': 'ADS_Mode', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|31]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_mode(uint8_t* data, Ads1_111::Ads_modeType ads_mode);

  // config detail: {'description': 'ADS target acceleration for transmission',
  // 'offset': -7.0, 'precision': 0.05, 'len': 8, 'name': 'ADS_TarAcce',
  // 'is_signed_var': False, 'physical_range': '[-7|5.75]', 'bit': 15,
  // 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s2'}
  void set_p_ads_taracce(uint8_t* data, double ads_taracce);

  // config detail: {'description': 'ACC request ESP drive off', 'enum': {0:
  // 'ADS_DRIVEOFF_REQ_NO_DEMAND', 1: 'ADS_DRIVEOFF_REQ_DEMAND'}, 'precision':
  // 1.0, 'len': 1, 'name': 'ADS_Driveoff_Req', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_ads_driveoff_req(uint8_t* data,
                              Ads1_111::Ads_driveoff_reqType ads_driveoff_req);

  // config detail: {'description': 'target deceleration value from AEB',
  // 'offset': -16.0, 'precision': 0.000488, 'len': 16, 'name':
  // 'ADS_AEB_TarAcce', 'is_signed_var': False, 'physical_range': '[-16|16]',
  // 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s2'}
  void set_p_ads_aeb_taracce(uint8_t* data, double ads_aeb_taracce);

  // config detail: {'description': 'Request of the AEB deceleration control.',
  // 'enum': {0: 'ADS_AEB_TGTDECEL_REQ_NO_DEMAND', 1:
  // 'ADS_AEB_TGTDECEL_REQ_DEMAND'}, 'precision': 1.0, 'len': 1, 'name':
  // 'ADS_AEB_TgtDecel_Req', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_aeb_tgtdecel_req(
      uint8_t* data, Ads1_111::Ads_aeb_tgtdecel_reqType ads_aeb_tgtdecel_req);

 private:
  Ads1_111::Ads_dectostopType ads_dectostop_;
  Ads1_111::Ads_modeType ads_mode_;
  double ads_taracce_ = 0.0;
  Ads1_111::Ads_driveoff_reqType ads_driveoff_req_;
  double ads_aeb_taracce_ = 0.0;
  Ads1_111::Ads_aeb_tgtdecel_reqType ads_aeb_tgtdecel_req_;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
