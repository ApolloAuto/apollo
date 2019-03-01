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

#include "modules/canbus/vehicle/wey/protocol/ads3_38e.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

const int32_t Ads338e::ID = 0x38E;

// public
Ads338e::Ads338e() { Reset(); }

uint32_t Ads338e::GetPeriod() const {
  // TODO(ChaoMa) :modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Ads338e::UpdateData(uint8_t* data) {
  set_p_ads_bcm_worksts(data, ads_bcm_worksts_);
  set_p_ads_bcmworkstsvalid(data, ads_bcmworkstsvalid_);
  set_p_ads_reqcontrolbcm(data, ads_reqcontrolbcm_);
  set_p_highbeamton(data, highbeamton_);
  set_p_dippedbeamon(data, dippedbeamon_);
  set_p_turnllighton(data, turnllighton_);
  set_p_emergencylighton(data, emergencylighton_);
  set_p_ffoglampon(data, ffoglampon_);
  set_p_rfoglampon(data, rfoglampon_);
  set_p_brakelight(data, brakelight_);
  set_p_hornon(data, hornon_);
  set_p_fwindshieldwiper(data, fwindshieldwiper_);
  set_p_rwindshieldwiper(data, rwindshieldwiper_);
}

void Ads338e::Reset() {
  // TODO(ChaoMa) you should check this manually
  ads_bcm_worksts_ = Ads3_38e::ADS_BCM_WORKSTS_DISABLE;
  ads_bcmworkstsvalid_ = Ads3_38e::ADS_BCMWORKSTSVALID_INVALID;
  ads_reqcontrolbcm_ = Ads3_38e::ADS_REQCONTROLBCM_NO_REQUEST;
  highbeamton_ = Ads3_38e::HIGHBEAMTON_TURN_OFF;
  dippedbeamon_ = Ads3_38e::DIPPEDBEAMON_TURN_OFF;
  turnllighton_ = Ads3_38e::TURNLLIGHTON_TURN_OFF;
  emergencylighton_ = Ads3_38e::EMERGENCYLIGHTON_TURN_OFF;
  ffoglampon_ = Ads3_38e::FFOGLAMPON_TURN_OFF;
  rfoglampon_ = Ads3_38e::RFOGLAMPON_TURN_OFF;
  brakelight_ = Ads3_38e::BRAKELIGHT_TURN_OFF;
  hornon_ = Ads3_38e::HORNON_TURN_OFF;
  fwindshieldwiper_ = Ads3_38e::FWINDSHIELDWIPER_TURN_OFF;
  rwindshieldwiper_ = Ads3_38e::RWINDSHIELDWIPER_TURN_OFF;
}

Ads338e* Ads338e::set_ads_bcm_worksts(
    Ads3_38e::Ads_bcm_workstsType ads_bcm_worksts) {
  ads_bcm_worksts_ = ads_bcm_worksts;
  return this;
}

// config detail: {'description': 'The work status of ADS control unit.
// This signal should be contained in every message sent by ADS.', 'enum':
// {0: 'ADS_BCM_WORKSTS_DISABLE', 1: 'ADS_BCM_WORKSTS_ENABLE',
// 2: 'ADS_BCM_WORKSTS_ACTIVE', 3: 'ADS_BCM_WORKSTS_FAILED'}, 'precision': 1.0,
// 'len': 2, 'name': 'ADS_BCM_WorkSts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_ads_bcm_worksts(
    uint8_t* data, Ads3_38e::Ads_bcm_workstsType ads_bcm_worksts) {
  int x = ads_bcm_worksts;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 5, 2);
}

Ads338e* Ads338e::set_ads_bcmworkstsvalid(
    Ads3_38e::Ads_bcmworkstsvalidType ads_bcmworkstsvalid) {
  ads_bcmworkstsvalid_ = ads_bcmworkstsvalid;
  return this;
}

// config detail: {'description': 'This Signal reflect the ADS_WorkSts Signal
// is Valid OR not', 'enum': {0: 'ADS_BCMWORKSTSVALID_INVALID',
// 1: 'ADS_BCMWORKSTSVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'ADS_BCMWorkStsValid','is_signed_var': False, 'offset': 0.0,'physical_range':
// '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ads338e::set_p_ads_bcmworkstsvalid(
    uint8_t* data, Ads3_38e::Ads_bcmworkstsvalidType ads_bcmworkstsvalid) {
  int x = ads_bcmworkstsvalid;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

Ads338e* Ads338e::set_ads_reqcontrolbcm(
    Ads3_38e::Ads_reqcontrolbcmType ads_reqcontrolbcm) {
  ads_reqcontrolbcm_ = ads_reqcontrolbcm;
  return this;
}

// config detail: {'description': 'Vehicle Task request.If ADS request control
// BCM', 'enum': {0: 'ADS_REQCONTROLBCM_NO_REQUEST', 1:
// 'ADS_REQCONTROLBCM_REQUEST'}, 'precision': 1.0, 'len': 1, 'name':
// 'ADS_ReqControlBCM', 'is_signed_var':False, 'offset': 0.0, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ads338e::set_p_ads_reqcontrolbcm(
    uint8_t* data, Ads3_38e::Ads_reqcontrolbcmType ads_reqcontrolbcm) {
  int x = ads_reqcontrolbcm;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 0, 1);
}

Ads338e* Ads338e::set_highbeamton(Ads3_38e::HighbeamtonType highbeamton) {
  highbeamton_ = highbeamton;
  return this;
}

// config detail: {'description': 'Open theHigh Beam light', 'enum': {0:
// 'HIGHBEAMTON_TURN_OFF', 1: 'HIGHBEAMTON_TURN_ON'}, 'precision': 1.0,
// 'len': 1, 'name': 'HighBeamtON', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_highbeamton(uint8_t* data,
                                Ads3_38e::HighbeamtonType highbeamton) {
  int x = highbeamton;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 3, 1);
}

Ads338e* Ads338e::set_dippedbeamon(Ads3_38e::DippedbeamonType dippedbeamon) {
  dippedbeamon_ = dippedbeamon;
  return this;
}

// config detail: {'description': 'Open the dipped Beam light', 'enum': {0:
// 'DIPPEDBEAMON_TURN_OFF', 1: 'DIPPEDBEAMON_TURN_ON'}, 'precision': 1.0,
// 'len': 1, 'name': 'DippedBeamON', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_dippedbeamon(uint8_t* data,
                                 Ads3_38e::DippedbeamonType dippedbeamon) {
  int x = dippedbeamon;

  Byte to_set(data + 1);
  to_set.set_value(static_cast<uint8_t>(x), 4, 1);
}

Ads338e* Ads338e::set_turnllighton(Ads3_38e::TurnllightonType turnllighton) {
  turnllighton_ = turnllighton;
  return this;
}

// config detail: {'description': 'Open the Turn light', 'enum': {0:
// 'TURNLLIGHTON_TURN_OFF', 1: 'TURNLLIGHTON_TURN_LEFT_ON', 2:
// 'TURNLLIGHTON_TURN_RIGHT_ON', 3: 'TURNLLIGHTON_RESERVED'}, 'precision': 1.0,
// 'len': 2, 'name': 'TurnlLightON', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_turnllighton(uint8_t* data,
                                 Ads3_38e::TurnllightonType turnllighton) {
  int x = turnllighton;

  Byte to_set(data + 2);
  to_set.set_value(static_cast<uint8_t>(x), 0, 2);
}

Ads338e* Ads338e::set_emergencylighton(
    Ads3_38e::EmergencylightonType emergencylighton) {
  emergencylighton_ = emergencylighton;
  return this;
}

// config detail: {'description': 'Open the emergency light', 'enum': {0:
// 'EMERGENCYLIGHTON_TURN_OFF', 1: 'EMERGENCYLIGHTON_TURN_ON'},'precision':1.0,
// 'len': 1, 'name': 'EmergencyLightON', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 45, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_emergencylighton(
    uint8_t* data, Ads3_38e::EmergencylightonType emergencylighton) {
  int x = emergencylighton;

  Byte to_set(data + 5);
  to_set.set_value(static_cast<uint8_t>(x), 5, 1);
}

Ads338e* Ads338e::set_ffoglampon(Ads3_38e::FfoglamponType ffoglampon) {
  ffoglampon_ = ffoglampon;
  return this;
}

// config detail: {'description': 'Open the front fog light', 'enum': {0:
// 'FFOGLAMPON_TURN_OFF', 1: 'FFOGLAMPON_TURN_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'FFogLampON', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 46, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_ffoglampon(uint8_t* data,
                               Ads3_38e::FfoglamponType ffoglampon) {
  int x = ffoglampon;

  Byte to_set(data + 5);
  to_set.set_value(static_cast<uint8_t>(x), 6, 1);
}

Ads338e* Ads338e::set_rfoglampon(Ads3_38e::RfoglamponType rfoglampon) {
  rfoglampon_ = rfoglampon;
  return this;
}

// config detail: {'description': 'Open the rear fog ligh', 'enum': {0:
// 'RFOGLAMPON_TURN_OFF', 1: 'RFOGLAMPON_TURN_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'RFogLampON', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_rfoglampon(uint8_t* data,
                               Ads3_38e::RfoglamponType rfoglampon) {
  int x = rfoglampon;

  Byte to_set(data + 5);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

Ads338e* Ads338e::set_brakelight(Ads3_38e::BrakelightType brakelight) {
  brakelight_ = brakelight;
  return this;
}

// config detail: {'description': 'Open the brake light', 'enum': {0:
// 'BRAKELIGHT_TURN_OFF', 1: 'BRAKELIGHT_TURN_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'BrakeLight', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 48, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_brakelight(uint8_t* data,
                               Ads3_38e::BrakelightType brakelight) {
  int x = brakelight;

  Byte to_set(data + 6);
  to_set.set_value(static_cast<uint8_t>(x), 0, 1);
}

Ads338e* Ads338e::set_hornon(Ads3_38e::HornonType hornon) {
  hornon_ = hornon;
  return this;
}

// config detail: {'description': 'Open the horn voice', 'enum': {0:
// 'HORNON_TURN_OFF', 1: 'HORNON_TURN_ON'}, 'precision': 1.0, 'len': 1, 'name':
// 'HornON', 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 49, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ads338e::set_p_hornon(uint8_t* data, Ads3_38e::HornonType hornon) {
  int x = hornon;

  Byte to_set(data + 6);
  to_set.set_value(static_cast<uint8_t>(x), 1, 1);
}

Ads338e* Ads338e::set_fwindshieldwiper(
    Ads3_38e::FwindshieldwiperType fwindshieldwiper) {
  fwindshieldwiper_ = fwindshieldwiper;
  return this;
}

// config detail: {'description': 'Open front window shield wiper', 'enum': {0:
// 'FWINDSHIELDWIPER_TURN_OFF', 1: 'FWINDSHIELDWIPER_TURN_ON'},'precision':1.0,
// 'len': 1, 'name': 'Fwindshieldwiper', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 50, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_fwindshieldwiper(
    uint8_t* data, Ads3_38e::FwindshieldwiperType fwindshieldwiper) {
  int x = fwindshieldwiper;

  Byte to_set(data + 6);
  to_set.set_value(static_cast<uint8_t>(x), 2, 1);
}

Ads338e* Ads338e::set_rwindshieldwiper(
    Ads3_38e::RwindshieldwiperType rwindshieldwiper) {
  rwindshieldwiper_ = rwindshieldwiper;
  return this;
}

// config detail: {'description': 'Open rear window shield wiper', 'enum': {0:
// 'RWINDSHIELDWIPER_TURN_OFF', 1: 'RWINDSHIELDWIPER_TURN_ON'},'precision':1.0,
// 'len': 1, 'name': 'Rwindshieldwiper', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 60, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
void Ads338e::set_p_rwindshieldwiper(
    uint8_t* data, Ads3_38e::RwindshieldwiperType rwindshieldwiper) {
  int x = rwindshieldwiper;

  Byte to_set(data + 7);
  to_set.set_value(static_cast<uint8_t>(x), 4, 1);
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
