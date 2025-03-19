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

#include "modules/canbus/vehicle/wey/protocol/status_310.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::drivers::canbus::Byte;

Status310::Status310() {}
const int32_t Status310::ID = 0x310;

void Status310::Parse(const std::uint8_t* bytes, int32_t length,
                      ChassisDetail* chassis) const {
  chassis->mutable_wey()->mutable_status_310()->set_longitudeaccvalid(
      longitudeaccvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_lateralaccevalid(
      lateralaccevalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_vehdynyawratevalid(
      vehdynyawratevalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_flwheelspdvalid(
      flwheelspdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_frwheelspdvalid(
      frwheelspdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_rlwheelspdvalid(
      rlwheelspdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_rrwheelspdvalid(
      rrwheelspdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_vehiclespdvalid(
      vehiclespdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_longitudedrivingmode(
      longitudedrivingmode(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_engspdvalid(
      engspdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_accepedaloverride(
      accepedaloverride(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_brakepedalstatus(
      brakepedalstatus(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_espbrakelightsts(
      espbrakelightsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_epbswtpositionvalid(
      epbswtpositionvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_epbsts(
      epbsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_currentgearvalid(
      currentgearvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_epstrqsnsrsts(
      epstrqsnsrsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_eps_interferdetdvalid(
      eps_interferdetdvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_epshandsdetnsts(
      epshandsdetnsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_eps_handsdetnstsvalid(
      eps_handsdetnstsvalid(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_steerwheelanglesign(
      steerwheelanglesign(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_steerwheelspdsign(
      steerwheelspdsign(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_driverdoorsts(
      driverdoorsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_rldoorsts(
      rldoorsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_passengerdoorsts(
      passengerdoorsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_rrdoorsts(
      rrdoorsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_frontfoglmpsts(
      frontfoglmpsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_rearfoglmpsts(
      rearfoglmpsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_lowbeamsts(
      lowbeamsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_highbeamsts(
      highbeamsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_leftturnlampsts(
      leftturnlampsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_rightturnlampsts(
      rightturnlampsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_bcm_availsts(
      bcm_availsts(bytes, length));
  chassis->mutable_wey()->mutable_status_310()->set_brakelmpsts(
      brakelmpsts(bytes, length));
  // Added for response check
  chassis->mutable_check_response()->set_is_esp_online(
      longitudedrivingmode(bytes, length) != 0);
  chassis->mutable_check_response()->set_is_vcu_online(
      longitudedrivingmode(bytes, length) != 0);
}

// config detail: {'description': 'Longitude acceleration valid', 'enum':
// {0: 'LONGITUDEACCVALID_INVALID', 1: 'LONGITUDEACCVALID_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'longitudeaccvalid', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::LongitudeaccvalidType Status310::longitudeaccvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Status_310::LongitudeaccvalidType ret =
      static_cast<Status_310::LongitudeaccvalidType>(x);
  return ret;
}

// config detail: {'description': 'Indicates Lateral Signal State',
// 'enum': {0: 'LATERALACCEVALID_INVALID', 1: 'LATERALACCEVALID_VALID'},
// 'precision': 1.0, 'len': 1,'name':'lateralaccevalid','is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::LateralaccevalidType Status310::lateralaccevalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Status_310::LateralaccevalidType ret =
      static_cast<Status_310::LateralaccevalidType>(x);
  return ret;
}

// config detail: {'description': 'Vehicle yaw rate valid', 'enum':
// {0: 'VEHDYNYAWRATEVALID_INVALID', 1: 'VEHDYNYAWRATEVALID_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'vehdynyawratevalid', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::VehdynyawratevalidType Status310::vehdynyawratevalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Status_310::VehdynyawratevalidType ret =
      static_cast<Status_310::VehdynyawratevalidType>(x);
  return ret;
}

// config detail: {'description': 'Front right wheel speed valid',
// 'enum': {0: 'FLWHEELSPDVALID_INVALID', 1: 'FLWHEELSPDVALID_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'flwheelspdvalid','is_signed_var':False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::FlwheelspdvalidType Status310::flwheelspdvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Status_310::FlwheelspdvalidType ret =
      static_cast<Status_310::FlwheelspdvalidType>(x);
  return ret;
}

// config detail: {'description': 'Front right wheel speed valid',
// 'enum': {0: 'FRWHEELSPDVALID_INVALID', 1: 'FRWHEELSPDVALID_VALID'},
// 'precision': 1.0, 'len': 1, 'name':'frwheelspdvalid','is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::FrwheelspdvalidType Status310::frwheelspdvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Status_310::FrwheelspdvalidType ret =
      static_cast<Status_310::FrwheelspdvalidType>(x);
  return ret;
}

// config detail: {'description': 'Rear left wheel speed valid', 'enum':
// {0: 'RLWHEELSPDVALID_INVALID', 1: 'RLWHEELSPDVALID_VALID'},'precision': 1.0,
// 'len': 1, 'name': 'rlwheelspdvalid', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::RlwheelspdvalidType Status310::rlwheelspdvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Status_310::RlwheelspdvalidType ret =
      static_cast<Status_310::RlwheelspdvalidType>(x);
  return ret;
}

// config detail: {'description': 'Rear right wheel speed valid', 'enum':
// {0: 'RRWHEELSPDVALID_INVALID', 1: 'RRWHEELSPDVALID_VALID'},'precision': 1.0,
// 'len': 1, 'name': 'rrwheelspdvalid', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::RrwheelspdvalidType Status310::rrwheelspdvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Status_310::RrwheelspdvalidType ret =
      static_cast<Status_310::RrwheelspdvalidType>(x);
  return ret;
}

// config detail: {'description': 'Quality/fault information to current Vehicle
// speed information', 'enum': {0: 'VEHICLESPDVALID_INVALID',
// 1: 'VEHICLESPDVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'vehiclespdvalid', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Status_310::VehiclespdvalidType Status310::vehiclespdvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Status_310::VehiclespdvalidType ret =
      static_cast<Status_310::VehiclespdvalidType>(x);
  return ret;
}

// config detail: {'description': 'This signal indicates if ECM control for ADS
// torque request is active or not.', 'enum':
// {0: 'LONGITUDEDRIVINGMODE_MANUALMODE',
// 1: 'LONGITUDEDRIVINGMODE_AUTOMATICSTANDBY',
// 2: 'LONGITUDEDRIVINGMODE_AUTOMATICACCELERATION',
// 3: 'LONGITUDEDRIVINGMODE_AUTOMATICDECELERATION'}, 'precision': 1.0,'len': 2,
// 'name': 'longitudedrivingmode', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 14, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::LongitudedrivingmodeType Status310::longitudedrivingmode(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 2);

  Status_310::LongitudedrivingmodeType ret =
      static_cast<Status_310::LongitudedrivingmodeType>(x);
  return ret;
}

// config detail: {'description': 'Engine speed valid', 'enum':
// {0: 'ENGSPDVALID_INVALID',1: 'ENGSPDVALID_VALID',2:'ENGSPDVALID_INIT_VALUE',
// 3: 'ENGSPDVALID_RESERVED'},'precision': 1.0,'len': 2, 'name': 'engspdvalid',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 12,
// 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Status_310::EngspdvalidType Status310::engspdvalid(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 2);

  Status_310::EngspdvalidType ret = static_cast<Status_310::EngspdvalidType>(x);
  return ret;
}

// config detail: {'description': 'Detect Acceleration Pedal Override', 'enum':
// {0: 'ACCEPEDALOVERRIDE_NOT_OVERRIDE', 1: 'ACCEPEDALOVERRIDE_OVERRIDE'},
// 'precision': 1.0, 'len': 1, 'name': 'accepedaloverride', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::AccepedaloverrideType Status310::accepedaloverride(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  Status_310::AccepedaloverrideType ret =
      static_cast<Status_310::AccepedaloverrideType>(x);
  return ret;
}

// config detail: {'description': 'indicates the brake pedal is pressed or not
// or incorrect for plausibility check.', 'enum':
// {0: 'BRAKEPEDALSTATUS_NOT_PRESSED', 1: 'BRAKEPEDALSTATUS_PRESSED',
// 2: 'BRAKEPEDALSTATUS_RESERVED1', 3: 'BRAKEPEDALSTATUS_ERROR'},
// 'precision': 1.0, 'len': 2, 'name': 'brakepedalstatus',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]',
// 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Status_310::BrakepedalstatusType Status310::brakepedalstatus(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Status_310::BrakepedalstatusType ret =
      static_cast<Status_310::BrakepedalstatusType>(x);
  return ret;
}

// config detail: {'description': 'Brake light lamp(on/off),come from ESP',
// 'enum': {0: 'ESPBRAKELIGHTSTS_OFF', 1: 'ESPBRAKELIGHTSTS_ON'},
// 'precision': 1.0, 'len': 1, 'name': 'espbrakelightsts', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::EspbrakelightstsType Status310::espbrakelightsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  Status_310::EspbrakelightstsType ret =
      static_cast<Status_310::EspbrakelightstsType>(x);
  return ret;
}

// config detail: {'description': 'EPB switch position signal valid', 'enum':
// {0: 'EPBSWTPOSITIONVALID_VALID', 1: 'EPBSWTPOSITIONVALID_NOT_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'epbswtpositionvalid',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]',
// 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Status_310::EpbswtpositionvalidType Status310::epbswtpositionvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  Status_310::EpbswtpositionvalidType ret =
      static_cast<Status_310::EpbswtpositionvalidType>(x);
  return ret;
}

// config detail: {'description': 'EPB status', 'enum': {0: 'EPBSTS_RELEASED',
// 1: 'EPBSTS_CLOSED', 2: 'EPBSTS_IN_PROGRESS', 3: 'EPBSTS_UNKNOWN'},
// 'precision': 1.0, 'len': 2, 'name': 'epbsts', 'is_signed_var': False,
// 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::EpbstsType Status310::epbsts(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 2);

  Status_310::EpbstsType ret = static_cast<Status_310::EpbstsType>(x);
  return ret;
}

// config detail: {'description': 'Current gear valid', 'enum':
// {0: 'CURRENTGEARVALID_INVALID', 1: 'CURRENTGEARVALID_VALID'},
// 'precision': 1.0, 'len': 1, 'name': 'currentgearvalid', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 25, 'type':
// 'enum', 'order': 'motorola', 'physical_unit': ''}
Status_310::CurrentgearvalidType Status310::currentgearvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  Status_310::CurrentgearvalidType ret =
      static_cast<Status_310::CurrentgearvalidType>(x);
  return ret;
}

// config detail: {'description': 'EPS torque sensor status', 'enum':
// {0: 'EPSTRQSNSRSTS_NORMAL', 1: 'EPSTRQSNSRSTS_ABNORMAL'}, 'precision': 1.0,
// 'len': 1, 'name': 'epstrqsnsrsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::EpstrqsnsrstsType Status310::epstrqsnsrsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  Status_310::EpstrqsnsrstsType ret =
      static_cast<Status_310::EpstrqsnsrstsType>(x);
  return ret;
}

// config detail: {'description': 'Driver Steering Interference Detected
// Validity', 'enum': {0: 'EPS_INTERFERDETDVALID_INVALID',
// 1: 'EPS_INTERFERDETDVALID_VALID'}, 'precision': 1.0, 'len': 1,
// 'name': 'eps_interferdetdvalid', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::Eps_interferdetdvalidType Status310::eps_interferdetdvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  Status_310::Eps_interferdetdvalidType ret =
      static_cast<Status_310::Eps_interferdetdvalidType>(x);
  return ret;
}

// config detail: {'description': 'Hands Off Steering Wheel Detection status',
// 'enum': {0: 'EPSHANDSDETNSTS_HANDSOFF_NOT_DETECTED',
// 1: 'EPSHANDSDETNSTS_HANDOFFF_DETECTED'}, 'precision': 1.0, 'len': 1,
// 'name': 'epshandsdetnsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 27, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::EpshandsdetnstsType Status310::epshandsdetnsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  Status_310::EpshandsdetnstsType ret =
      static_cast<Status_310::EpshandsdetnstsType>(x);
  return ret;
}

// config detail: {'description': 'Hands Off Steering Wheel Detection status
// Validity', 'enum': {0: 'EPS_HANDSDETNSTSVALID_INVALID',
// 1: 'EPS_HANDSDETNSTSVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
// 'eps_handsdetnstsvalid', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::Eps_handsdetnstsvalidType Status310::eps_handsdetnstsvalid(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Status_310::Eps_handsdetnstsvalidType ret =
      static_cast<Status_310::Eps_handsdetnstsvalidType>(x);
  return ret;
}

// config detail: {'description': 'sign of steering wheel angle', 'enum':
// {0: 'STEERWHEELANGLESIGN_LEFT_POSITIVE',
// 1: 'STEERWHEELANGLESIGN_RIGHT_NEGATIVE'}, 'precision': 1.0, 'len': 1,
// 'name': 'steerwheelanglesign', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::SteerwheelanglesignType Status310::steerwheelanglesign(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Status_310::SteerwheelanglesignType ret =
      static_cast<Status_310::SteerwheelanglesignType>(x);
  return ret;
}

// config detail: {'description': 'sign of steering wheel speed ', 'enum':
// {0: 'STEERWHEELSPDSIGN_LEFT_POSITIVE',1:'STEERWHEELSPDSIGN_RIGHT_NEGATIVE'},
// 'precision': 1.0, 'len': 1, 'name': 'steerwheelspdsign', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum',
// 'order': 'motorola', 'physical_unit': ''}
Status_310::SteerwheelspdsignType Status310::steerwheelspdsign(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  Status_310::SteerwheelspdsignType ret =
      static_cast<Status_310::SteerwheelspdsignType>(x);
  return ret;
}

// config detail: {'description': 'Driver door status.', 'enum':
// {0: 'DRIVERDOORSTS_CLOSED', 1: 'DRIVERDOORSTS_OPEN'}, 'precision': 1.0,
// 'len': 1, 'name': 'driverdoorsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::DriverdoorstsType Status310::driverdoorsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  Status_310::DriverdoorstsType ret =
      static_cast<Status_310::DriverdoorstsType>(x);
  return ret;
}

// config detail: {'description': 'Left rear door status', 'enum':
// {0: 'RLDOORSTS_CLOSED', 1: 'RLDOORSTS_OPEN'}, 'precision': 1.0, 'len': 1,
// 'name': 'rldoorsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::RldoorstsType Status310::rldoorsts(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Status_310::RldoorstsType ret = static_cast<Status_310::RldoorstsType>(x);
  return ret;
}

// config detail: {'description': 'Passenger door status.', 'enum':
// {0: 'PASSENGERDOORSTS_CLOSED', 1: 'PASSENGERDOORSTS_OPEN'},'precision': 1.0,
// 'len': 1, 'name': 'passengerdoorsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 45, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::PassengerdoorstsType Status310::passengerdoorsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 1);

  Status_310::PassengerdoorstsType ret =
      static_cast<Status_310::PassengerdoorstsType>(x);
  return ret;
}

// config detail: {'description': 'Right   rear door status', 'enum':
// {0: 'RRDOORSTS_CLOSED', 1: 'RRDOORSTS_OPEN'}, 'precision': 1.0, 'len': 1,
// 'name': 'rrdoorsts', 'is_signed_var': False, 'offset': 0.0,'physical_range':
// '[0|1]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit':''}
Status_310::RrdoorstsType Status310::rrdoorsts(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  Status_310::RrdoorstsType ret = static_cast<Status_310::RrdoorstsType>(x);
  return ret;
}

// config detail: {'description': 'Front fog lamp status', 'enum':
// {0: 'FRONTFOGLMPSTS_OFF', 1:'FRONTFOGLMPSTS_ON',2:'FRONTFOGLMPSTS_RESERVED',
// 3: 'FRONTFOGLMPSTS_NOT_AVAILABLE'}, 'precision': 1.0, 'len': 2, 'name':
// 'frontfoglmpsts', 'is_signed_var': False, 'offset': 0.0, 'physical_range':
// '[0|3]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit':''}
Status_310::FrontfoglmpstsType Status310::frontfoglmpsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 2);

  Status_310::FrontfoglmpstsType ret =
      static_cast<Status_310::FrontfoglmpstsType>(x);
  return ret;
}

// config detail: {'description': 'Rear fog lamp status', 'enum':
// {0: 'REARFOGLMPSTS_OFF', 1: 'REARFOGLMPSTS_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'rearfoglmpsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 51, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::RearfoglmpstsType Status310::rearfoglmpsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 1);

  Status_310::RearfoglmpstsType ret =
      static_cast<Status_310::RearfoglmpstsType>(x);
  return ret;
}

// config detail: {'description': 'Low beam status', 'enum':
// {0: 'LOWBEAMSTS_OFF', 1: 'LOWBEAMSTS_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'lowbeamsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 49, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::LowbeamstsType Status310::lowbeamsts(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(1, 1);

  Status_310::LowbeamstsType ret = static_cast<Status_310::LowbeamstsType>(x);
  return ret;
}

// config detail: {'description': 'High beam status', 'enum':
// {0: 'HIGHBEAMSTS_OFF', 1: 'HIGHBEAMSTS_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'highbeamsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 63, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::HighbeamstsType Status310::highbeamsts(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(7, 1);

  Status_310::HighbeamstsType ret = static_cast<Status_310::HighbeamstsType>(x);
  return ret;
}

// config detail: {'description': 'Left turn lamp status', 'enum':
// {0: 'LEFTTURNLAMPSTS_OFF', 1: 'LEFTTURNLAMPSTS_ON'}, 'precision': 1.0,
// 'len': 1, 'name': 'leftturnlampsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 62, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::LeftturnlampstsType Status310::leftturnlampsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 1);

  Status_310::LeftturnlampstsType ret =
      static_cast<Status_310::LeftturnlampstsType>(x);
  return ret;
}

// config detail: {'description': 'Right turn lamp status', 'enum':
// {0: 'RIGHTTURNLAMPSTS_OFF', 1: 'RIGHTTURNLAMPSTS_ON'}, 'precision': 1.0,
// 'len': 1, 'name': 'rightturnlampsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 60, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::RightturnlampstsType Status310::rightturnlampsts(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  Status_310::RightturnlampstsType ret =
      static_cast<Status_310::RightturnlampstsType>(x);
  return ret;
}

// config detail: {'description': 'The work status of BCM', 'enum':
// {0: 'BCM_AVAILSTS_MANUAL_MODE', 1: 'BCM_AVAILSTS_AUTONOMOUS_MODE',
// 2: 'BCM_AVAILSTS_RESERVED1', 3: 'BCM_AVAILSTS_RESERVED2'}, 'precision': 1.0,
// 'len': 2, 'name': 'bcm_availsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|3]', 'bit': 58, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::Bcm_availstsType Status310::bcm_availsts(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 2);

  Status_310::Bcm_availstsType ret =
      static_cast<Status_310::Bcm_availstsType>(x);
  return ret;
}

// config detail: {'description': 'Break Lamp status', 'enum':
// {0: 'BRAKELMPSTS_OFF', 1: 'BRAKELMPSTS_ON'}, 'precision': 1.0, 'len': 1,
// 'name': 'brakelmpsts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum', 'order': 'motorola',
// 'physical_unit': ''}
Status_310::BrakelmpstsType Status310::brakelmpsts(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  Status_310::BrakelmpstsType ret = static_cast<Status_310::BrakelmpstsType>(x);
  return ret;
}
}  // namespace wey
}  // namespace canbus
}  // namespace apollo
