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

class Status310 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Status310();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'description': 'Longitude acceleration valid', 'enum':
  // {0: 'LONGITUDEACCVALID_INVALID', 1: 'LONGITUDEACCVALID_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'LongitudeAccValid', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::LongitudeaccvalidType longitudeaccvalid(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Indicates Lateral Signal State', 'enum':
  // {0: 'LATERALACCEVALID_INVALID', 1: 'LATERALACCEVALID_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'LateralAcceValid', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::LateralaccevalidType lateralaccevalid(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'description': 'Vehicle yaw rate valid', 'enum':
  // {0: 'VEHDYNYAWRATEVALID_INVALID', 1: 'VEHDYNYAWRATEVALID_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'VehDynYawRateValid', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::VehdynyawratevalidType vehdynyawratevalid(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed valid', 'enum':
  // {0: 'FLWHEELSPDVALID_INVALID', 1:'FLWHEELSPDVALID_VALID'},'precision':1.0,
  // 'len': 1, 'name': 'FLWheelSpdValid', 'is_signed_var': False, 'offset':0.0,
  // 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::FlwheelspdvalidType flwheelspdvalid(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Front right wheel speed valid', 'enum':
  // {0: 'FRWHEELSPDVALID_INVALID', 1:'FRWHEELSPDVALID_VALID'},'precision':1.0,
  // 'len': 1, 'name': 'FRWheelSpdValid', 'is_signed_var': False, 'offset':0.0,
  // 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::FrwheelspdvalidType frwheelspdvalid(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Rear left wheel speed valid', 'enum':
  // {0: 'RLWHEELSPDVALID_INVALID', 1: 'RLWHEELSPDVALID_VALID'}, 'precision':
  // 1.0, 'len': 1, 'name': 'RLWheelSpdValid', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::RlwheelspdvalidType rlwheelspdvalid(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Rear right wheel speed valid', 'enum':
  // {0: 'RRWHEELSPDVALID_INVALID', 1: 'RRWHEELSPDVALID_VALID'}, 'precision':
  // 1.0, 'len': 1, 'name': 'RRWheelSpdValid', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::RrwheelspdvalidType rrwheelspdvalid(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Quality/fault information to current
  // Vehicle speed information', 'enum': {0: 'VEHICLESPDVALID_INVALID',
  // 1: 'VEHICLESPDVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'VehicleSpdValid', 'is_signed_var': False, 'offset': 0.0,'physical_range':
  // '[0|1]', 'bit': 0, 'type': 'enum', 'order':'motorola','physical_unit': ''}
  Status_310::VehiclespdvalidType vehiclespdvalid(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'This signal indicates if ECM control for
  // ADS torque request is active or not.', 'enum':
  // {0: 'LONGITUDEDRIVINGMODE_MANUALMODE',
  // 1: 'LONGITUDEDRIVINGMODE_AUTOMATICSTANDBY',
  // 2: 'LONGITUDEDRIVINGMODE_AUTOMATICACCELERATION',
  // 3: 'LONGITUDEDRIVINGMODE_AUTOMATICDECELERATION'}, 'precision': 1.0,
  // 'len': 2, 'name': 'LongitudeDrivingMode', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 14, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::LongitudedrivingmodeType longitudedrivingmode(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Engine speed valid', 'enum':
  // {0: 'ENGSPDVALID_INVALID', 1: 'ENGSPDVALID_VALID',
  // 2: 'ENGSPDVALID_INIT_VALUE', 3: 'ENGSPDVALID_RESERVED'}, 'precision': 1.0,
  // 'len': 2, 'name': 'EngSpdValid', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|2]', 'bit': 12, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::EngspdvalidType engspdvalid(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail:{'description': 'Detect Acceleration Pedal Override','enum':
  // {0: 'ACCEPEDALOVERRIDE_NOT_OVERRIDE', 1: 'ACCEPEDALOVERRIDE_OVERRIDE'},
  // 'precision': 1.0, 'len': 1, 'name': 'AccePedalOverride', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type':'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::AccepedaloverrideType accepedaloverride(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'indicates the brake pedal is pressed or
  // not or incorrect for plausibility check.', 'enum':
  // {0: 'BRAKEPEDALSTATUS_NOT_PRESSED', 1: 'BRAKEPEDALSTATUS_PRESSED',
  // 2: 'BRAKEPEDALSTATUS_RESERVED1', 3: 'BRAKEPEDALSTATUS_ERROR'},
  // 'precision': 1.0, 'len': 2, 'name': 'BrakePedalStatus', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::BrakepedalstatusType brakepedalstatus(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'description': 'Brake light lamp(on/off),come from ESP',
  // 'enum': {0: 'ESPBRAKELIGHTSTS_OFF', 1: 'ESPBRAKELIGHTSTS_ON'},'precision':
  // 1.0, 'len': 1, 'name': 'ESPBrakeLightSts', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::EspbrakelightstsType espbrakelightsts(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'description': 'EPB switch position signal valid', 'enum':
  // {0: 'EPBSWTPOSITIONVALID_VALID', 1: 'EPBSWTPOSITIONVALID_NOT_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'EPBSwtPositionValid', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::EpbswtpositionvalidType epbswtpositionvalid(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'EPB status', 'enum': {0:'EPBSTS_RELEASED',
  // 1: 'EPBSTS_CLOSED', 2: 'EPBSTS_IN_PROGRESS', 3: 'EPBSTS_UNKNOWN'},
  // 'precision': 1.0, 'len': 2, 'name': 'EPBSts', 'is_signed_var': False,
  // 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::EpbstsType epbsts(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'description': 'Current gear valid', 'enum':
  // {0: 'CURRENTGEARVALID_INVALID', 1: 'CURRENTGEARVALID_VALID'},
  // 'precision': 1.0, 'len': 1, 'name': 'CurrentGearValid', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 25,'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::CurrentgearvalidType currentgearvalid(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'description': 'EPS torque sensor status', 'enum':
  // {0: 'EPSTRQSNSRSTS_NORMAL', 1: 'EPSTRQSNSRSTS_ABNORMAL'}, 'precision':1.0,
  // 'len': 1, 'name': 'EPSTrqSnsrSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::EpstrqsnsrstsType epstrqsnsrsts(const std::uint8_t* bytes,
                                              const int32_t length) const;

  // config detail: {'description': 'Driver Steering Interference Detected
  // Validity', 'enum': {0: 'EPS_INTERFERDETDVALID_INVALID',
  // 1: 'EPS_INTERFERDETDVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'EPS_InterferDetdValid', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::Eps_interferdetdvalidType eps_interferdetdvalid(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Hands Off Steering Wheel Detection status',
  // 'enum': {0: 'EPSHANDSDETNSTS_HANDSOFF_NOT_DETECTED',
  // 1: 'EPSHANDSDETNSTS_HANDOFFF_DETECTED'}, 'precision': 1.0, 'len': 1,
  // 'name': 'EPSHandsDetnSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 27, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::EpshandsdetnstsType epshandsdetnsts(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Hands Off Steering Wheel Detection status
  // Validity', 'enum': {0: 'EPS_HANDSDETNSTSVALID_INVALID',
  // 1: 'EPS_HANDSDETNSTSVALID_VALID'}, 'precision': 1.0, 'len': 1, 'name':
  // 'EPS_HandsDetnStsValid', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::Eps_handsdetnstsvalidType eps_handsdetnstsvalid(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'sign of steering wheel angle', 'enum':
  // {0: 'STEERWHEELANGLESIGN_LEFT_POSITIVE',
  // 1: 'STEERWHEELANGLESIGN_RIGHT_NEGATIVE'}, 'precision': 1.0, 'len': 1,
  // 'name': 'SteerWheelAngleSign', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::SteerwheelanglesignType steerwheelanglesign(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'sign of steering wheel speed ', 'enum':
  // {0: 'STEERWHEELSPDSIGN_LEFT_POSITIVE',
  // 1: 'STEERWHEELSPDSIGN_RIGHT_NEGATIVE'}, 'precision': 1.0, 'len': 1,
  // 'name': 'SteerWheelSpdSign', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::SteerwheelspdsignType steerwheelspdsign(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'description': 'Driver door status.', 'enum':
  // {0: 'DRIVERDOORSTS_CLOSED', 1: 'DRIVERDOORSTS_OPEN'}, 'precision': 1.0,
  // 'len': 1, 'name': 'DriverDoorSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::DriverdoorstsType driverdoorsts(const std::uint8_t* bytes,
                                              const int32_t length) const;

  // config detail: {'description': 'Left rear door status', 'enum':
  // {0: 'RLDOORSTS_CLOSED', 1: 'RLDOORSTS_OPEN'}, 'precision': 1.0, 'len': 1,
  // 'name': 'RLDoorSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::RldoorstsType rldoorsts(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'description': 'Passenger door status.', 'enum':
  // {0: 'PASSENGERDOORSTS_CLOSED',1:'PASSENGERDOORSTS_OPEN'},'precision': 1.0,
  // 'len': 1, 'name': 'PassengerDoorSts','is_signed_var': False,'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 45, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::PassengerdoorstsType passengerdoorsts(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'description': 'Right   rear door status', 'enum':
  // {0: 'RRDOORSTS_CLOSED', 1: 'RRDOORSTS_OPEN'}, 'precision': 1.0, 'len': 1,
  // 'name': 'RRDoorSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 44, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::RrdoorstsType rrdoorsts(const std::uint8_t* bytes,
                                      const int32_t length) const;

  // config detail: {'description': 'Front fog lamp status', 'enum':
  // {0: 'FRONTFOGLMPSTS_OFF', 1: 'FRONTFOGLMPSTS_ON',
  // 2: 'FRONTFOGLMPSTS_RESERVED', 3: 'FRONTFOGLMPSTS_NOT_AVAILABLE'},
  // 'precision': 1.0, 'len': 2, 'name':'FrontFogLmpSts','is_signed_var':False,
  // 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 43, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Status_310::FrontfoglmpstsType frontfoglmpsts(const std::uint8_t* bytes,
                                                const int32_t length) const;

  // config detail: {'description': 'Rear fog lamp status', 'enum':
  // {0: 'REARFOGLMPSTS_OFF', 1: 'REARFOGLMPSTS_ON'}, 'precision':1.0,'len': 1,
  // 'name': 'RearFogLmpSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 51, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::RearfoglmpstsType rearfoglmpsts(const std::uint8_t* bytes,
                                              const int32_t length) const;

  // config detail: {'description': 'Low beam status', 'enum':
  // {0: 'LOWBEAMSTS_OFF', 1: 'LOWBEAMSTS_ON'}, 'precision': 1.0, 'len': 1,
  // 'name': 'LowBeamSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 49, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::LowbeamstsType lowbeamsts(const std::uint8_t* bytes,
                                        const int32_t length) const;

  // config detail: {'description': 'High beam status', 'enum':
  // {0: 'HIGHBEAMSTS_OFF', 1: 'HIGHBEAMSTS_ON'}, 'precision': 1.0, 'len': 1,
  // 'name': 'HighBeamSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 63, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::HighbeamstsType highbeamsts(const std::uint8_t* bytes,
                                          const int32_t length) const;

  // config detail: {'description': 'Left turn lamp status', 'enum':
  // {0: 'LEFTTURNLAMPSTS_OFF', 1: 'LEFTTURNLAMPSTS_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'LeftTurnLampSts', 'is_signed_var': False, 'offset':0.0,
  // 'physical_range': '[0|1]', 'bit': 62, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::LeftturnlampstsType leftturnlampsts(const std::uint8_t* bytes,
                                                  const int32_t length) const;

  // config detail: {'description': 'Right turn lamp status', 'enum':
  // {0: 'RIGHTTURNLAMPSTS_OFF', 1: 'RIGHTTURNLAMPSTS_ON'}, 'precision': 1.0,
  // 'len': 1, 'name': 'RightTurnLampSts', 'is_signed_var':False,'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 60, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::RightturnlampstsType rightturnlampsts(const std::uint8_t* bytes,
                                                    const int32_t length) const;

  // config detail: {'description': 'The work status of BCM', 'enum':
  // {0: 'BCM_AVAILSTS_MANUAL_MODE', 1: 'BCM_AVAILSTS_AUTONOMOUS_MODE',
  // 2: 'BCM_AVAILSTS_RESERVED1', 3: 'BCM_AVAILSTS_RESERVED2'},'precision':1.0,
  // 'len': 2, 'name': 'BCM_AvailSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|3]', 'bit': 58, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::Bcm_availstsType bcm_availsts(const std::uint8_t* bytes,
                                            const int32_t length) const;

  // config detail: {'description': 'Break Lamp status', 'enum':
  // {0: 'BRAKELMPSTS_OFF', 1: 'BRAKELMPSTS_ON'}, 'precision': 1.0, 'len': 1,
  // 'name': 'BrakeLmpSts', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Status_310::BrakelmpstsType brakelmpsts(const std::uint8_t* bytes,
                                          const int32_t length) const;
};

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
