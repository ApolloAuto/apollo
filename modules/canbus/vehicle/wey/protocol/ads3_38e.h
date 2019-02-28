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

class Ads338e : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Ads338e();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'The work status of ADS control unit.
  // This signal should be contained in every message sent by ADS.', 'enum':
  // {0: 'ADS_BCM_WORKSTS_DISABLE', 1: 'ADS_BCM_WORKSTS_ENABLE',
  // 2: 'ADS_BCM_WORKSTS_ACTIVE', 3:'ADS_BCM_WORKSTS_FAILED'},'precision': 1.0,
  // 'len': 2, 'name': 'ADS_BCM_WorkSts', 'is_signed_var': False,'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_ads_bcm_worksts(Ads3_38e::Ads_bcm_workstsType ads_bcm_worksts);

  // config detail: {'description': 'This Signal reflect the ADS_WorkSts Signal
  // is Valid OR not', 'enum': {0: 'ADS_BCMWORKSTSVALID_INVALID',
  // 1: 'ADS_BCMWORKSTSVALID_VALID'}, 'precision': 1.0, 'len': 1,
  // 'name': 'ADS_BCMWorkStsValid', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_ads_bcmworkstsvalid(
      Ads3_38e::Ads_bcmworkstsvalidType ads_bcmworkstsvalid);

  // config detail: {'description': 'Vehicle Task request.If ADS request
  // control BCM', 'enum': {0: 'ADS_REQCONTROLBCM_NO_REQUEST',
  // 1: 'ADS_REQCONTROLBCM_REQUEST'}, 'precision': 1.0, 'len': 1, 'name':
  // 'ADS_ReqControlBCM', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_ads_reqcontrolbcm(
      Ads3_38e::Ads_reqcontrolbcmType ads_reqcontrolbcm);

  // config detail: {'description': 'Open theHigh Beam light', 'enum':
  // {0: 'HIGHBEAMTON_TURN_OFF', 1: 'HIGHBEAMTON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'HighBeamtON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_highbeamton(Ads3_38e::HighbeamtonType highbeamton);

  // config detail: {'description': 'Open the dipped Beam light', 'enum':
  // {0: 'DIPPEDBEAMON_TURN_OFF', 1: 'DIPPEDBEAMON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'DippedBeamON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_dippedbeamon(Ads3_38e::DippedbeamonType dippedbeamon);

  // config detail: {'description': 'Open the Turn light', 'enum':
  // {0: 'TURNLLIGHTON_TURN_OFF', 1: 'TURNLLIGHTON_TURN_LEFT_ON',
  // 2: 'TURNLLIGHTON_TURN_RIGHT_ON', 3: 'TURNLLIGHTON_RESERVED'},
  // 'precision': 1.0, 'len': 2, 'name': 'TurnlLightON', 'is_signed_var':False,
  // 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Ads338e* set_turnllighton(Ads3_38e::TurnllightonType turnllighton);

  // config detail: {'description': 'Open the emergency light', 'enum':
  // {0: 'EMERGENCYLIGHTON_TURN_OFF', 1: 'EMERGENCYLIGHTON_TURN_ON'},
  // 'precision': 1.0, 'len': 1, 'name': 'EmergencyLightON', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 45, 'type':'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Ads338e* set_emergencylighton(
      Ads3_38e::EmergencylightonType emergencylighton);

  // config detail: {'description': 'Open the front fog light', 'enum':
  // {0: 'FFOGLAMPON_TURN_OFF', 1: 'FFOGLAMPON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'FFogLampON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 46, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_ffoglampon(Ads3_38e::FfoglamponType ffoglampon);

  // config detail: {'description': 'Open the rear fog ligh', 'enum':
  // {0: 'RFOGLAMPON_TURN_OFF', 1: 'RFOGLAMPON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'RFogLampON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_rfoglampon(Ads3_38e::RfoglamponType rfoglampon);

  // config detail: {'description': 'Open the brake light', 'enum':
  // {0: 'BRAKELIGHT_TURN_OFF', 1: 'BRAKELIGHT_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BrakeLight', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_brakelight(Ads3_38e::BrakelightType brakelight);

  // config detail: {'description': 'Open the horn voice', 'enum':
  // {0: 'HORNON_TURN_OFF', 1: 'HORNON_TURN_ON'}, 'precision': 1.0, 'len': 1,
  // 'name': 'HornON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 49, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Ads338e* set_hornon(Ads3_38e::HornonType hornon);

  // config detail: {'description': 'Open front window shield wiper', 'enum':
  // {0: 'FWINDSHIELDWIPER_TURN_OFF', 1: 'FWINDSHIELDWIPER_TURN_ON'},
  // 'precision': 1.0, 'len': 1, 'name': 'Fwindshieldwiper', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 50, 'type':'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Ads338e* set_fwindshieldwiper(
      Ads3_38e::FwindshieldwiperType fwindshieldwiper);

  // config detail: {'description': 'Open rear window shield wiper', 'enum':
  // {0: 'RWINDSHIELDWIPER_TURN_OFF', 1: 'RWINDSHIELDWIPER_TURN_ON'},
  // 'precision': 1.0, 'len': 1, 'name': 'Rwindshieldwiper', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 60, 'type':'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Ads338e* set_rwindshieldwiper(
      Ads3_38e::RwindshieldwiperType rwindshieldwiper);

 private:
  // config detail: {'description': 'The work status of ADS control unit.
  // This signal should be contained in every message sent by ADS.', 'enum':
  // {0: 'ADS_BCM_WORKSTS_DISABLE', 1: 'ADS_BCM_WORKSTS_ENABLE',
  // 2: 'ADS_BCM_WORKSTS_ACTIVE', 3: 'ADS_BCM_WORKSTS_FAILED'},
  // 'precision': 1.0, 'len': 2, 'name': 'ADS_BCM_WorkSts', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_ads_bcm_worksts(uint8_t* data,
                             Ads3_38e::Ads_bcm_workstsType ads_bcm_worksts);

  // config detail: {'description': 'This Signal reflect the ADS_WorkSts Signal
  // is Valid OR not', 'enum': {0: 'ADS_BCMWORKSTSVALID_INVALID',
  // 1: 'ADS_BCMWORKSTSVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'ADS_BCMWorkStsValid', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_bcmworkstsvalid(
      uint8_t* data, Ads3_38e::Ads_bcmworkstsvalidType ads_bcmworkstsvalid);

  // config detail: {'description': 'Vehicle Task request.If ADS request
  // control BCM', 'enum': {0: 'ADS_REQCONTROLBCM_NO_REQUEST',
  // 1: 'ADS_REQCONTROLBCM_REQUEST'}, 'precision': 1.0, 'len': 1,
  // 'name': 'ADS_ReqControlBCM', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ads_reqcontrolbcm(
      uint8_t* data, Ads3_38e::Ads_reqcontrolbcmType ads_reqcontrolbcm);

  // config detail: {'description': 'Open theHigh Beam light', 'enum':
  // {0: 'HIGHBEAMTON_TURN_OFF', 1: 'HIGHBEAMTON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'HighBeamtON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_highbeamton(uint8_t* data, Ads3_38e::HighbeamtonType highbeamton);

  // config detail: {'description': 'Open the dipped Beam light', 'enum':
  // {0: 'DIPPEDBEAMON_TURN_OFF', 1: 'DIPPEDBEAMON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'DippedBeamON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_dippedbeamon(uint8_t* data,
                          Ads3_38e::DippedbeamonType dippedbeamon);

  // config detail: {'description': 'Open the Turn light', 'enum':
  // {0: 'TURNLLIGHTON_TURN_OFF', 1: 'TURNLLIGHTON_TURN_LEFT_ON',
  // 2: 'TURNLLIGHTON_TURN_RIGHT_ON', 3: 'TURNLLIGHTON_RESERVED'},
  // 'precision': 1.0, 'len': 2, 'name': 'TurnlLightON', 'is_signed_var':False,
  // 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_turnllighton(uint8_t* data,
                          Ads3_38e::TurnllightonType turnllighton);

  // config detail: {'description': 'Open the emergency light', 'enum':
  // {0: 'EMERGENCYLIGHTON_TURN_OFF', 1: 'EMERGENCYLIGHTON_TURN_ON'},
  // 'precision': 1.0, 'len': 1, 'name': 'EmergencyLightON',
  // 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
  // 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_emergencylighton(uint8_t* data,
                              Ads3_38e::EmergencylightonType emergencylighton);

  // config detail: {'description': 'Open the front fog light', 'enum':
  // {0: 'FFOGLAMPON_TURN_OFF', 1: 'FFOGLAMPON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'FFogLampON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 46, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_ffoglampon(uint8_t* data, Ads3_38e::FfoglamponType ffoglampon);

  // config detail: {'description': 'Open the rear fog ligh', 'enum':
  // {0: 'RFOGLAMPON_TURN_OFF', 1: 'RFOGLAMPON_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'RFogLampON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_rfoglampon(uint8_t* data, Ads3_38e::RfoglamponType rfoglampon);

  // config detail: {'description': 'Open the brake light', 'enum':
  // {0: 'BRAKELIGHT_TURN_OFF', 1: 'BRAKELIGHT_TURN_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'BrakeLight', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_brakelight(uint8_t* data, Ads3_38e::BrakelightType brakelight);

  // config detail: {'description': 'Open the horn voice', 'enum':
  // {0: 'HORNON_TURN_OFF', 1: 'HORNON_TURN_ON'}, 'precision': 1.0, 'len': 1,
  // 'name': 'HornON', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 49, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_hornon(uint8_t* data, Ads3_38e::HornonType hornon);

  // config detail: {'description': 'Open front window shield wiper', 'enum':
  // {0: 'FWINDSHIELDWIPER_TURN_OFF', 1: 'FWINDSHIELDWIPER_TURN_ON'},
  // 'precision': 1.0, 'len': 1, 'name': 'Fwindshieldwiper', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 50,'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_fwindshieldwiper(uint8_t* data,
                              Ads3_38e::FwindshieldwiperType fwindshieldwiper);

  // config detail: {'description': 'Open rear window shield wiper', 'enum':
  // {0: 'RWINDSHIELDWIPER_TURN_OFF', 1: 'RWINDSHIELDWIPER_TURN_ON'},
  // 'precision': 1.0, 'len': 1, 'name': 'Rwindshieldwiper', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 60,'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_rwindshieldwiper(uint8_t* data,
                              Ads3_38e::RwindshieldwiperType rwindshieldwiper);

 private:
  Ads3_38e::Ads_bcm_workstsType ads_bcm_worksts_;
  Ads3_38e::Ads_bcmworkstsvalidType ads_bcmworkstsvalid_;
  Ads3_38e::Ads_reqcontrolbcmType ads_reqcontrolbcm_;
  Ads3_38e::HighbeamtonType highbeamton_;
  Ads3_38e::DippedbeamonType dippedbeamon_;
  Ads3_38e::TurnllightonType turnllighton_;
  Ads3_38e::EmergencylightonType emergencylighton_;
  Ads3_38e::FfoglamponType ffoglampon_;
  Ads3_38e::RfoglamponType rfoglampon_;
  Ads3_38e::BrakelightType brakelight_;
  Ads3_38e::HornonType hornon_;
  Ads3_38e::FwindshieldwiperType fwindshieldwiper_;
  Ads3_38e::RwindshieldwiperType rwindshieldwiper_;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
