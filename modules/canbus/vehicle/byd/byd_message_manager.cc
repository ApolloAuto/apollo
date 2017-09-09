/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/byd/byd_message_menager.h"

#include "modules/canbus/vehicle/byd/protocol/accel_e5.h"
#include "modules/canbus/vehicle/byd/protocol/brake_e5.h"
#include "modules/canbus/vehicle/byd/protocol/brakeinfo_e5.h"
#include "modules/canbus/vehicle/byd/protocol/fuellevel_e5.h"
#include "modules/canbus/vehicle/byd/protocol/gear_e5.h"
#include "modules/canbus/vehicle/byd/protocol/gps_e5.h"
#include "modules/canbus/vehicle/byd/protocol/gyro_e5.h"
#include "modules/canbus/vehicle/byd/protocol/misc_e5.h"
#include "modules/canbus/vehicle/byd/protocol/steering_e5.h"
#include "modules/canbus/vehicle/byd/protocol/throttle_e5.h"
#include "modules/canbus/vehicle/byd/protocol/throttleinfo_e5.h"
#include "modules/canbus/vehicle/byd/protocol/tirepressure_e5.h"
#include "modules/canbus/vehicle/byd/protocol/turnsignal_e5.h"
#include "modules/canbus/vehicle/byd/protocol/version_e5.h"
#include "modules/canbus/vehicle/byd/protocol/wheelspeed_e5.h"

namespace apollo {
namespace canbus {
namespace byd {

BydMessageManager::BydMessageManager() {
  // TODO(Authors): verify which one is recv/sent
  AddSendProtocolData<BrakeE5, true>();
  AddSendProtocolData<ThrottleE5, true>();
  AddSendProtocolData<SteeringE5, true>();
  AddSendProtocolData<GearE5, true>();
  AddSendProtocolData<TurnsignalE5, true>();

  AddRecvProtocolData<BrakeE5, true>();
  AddRecvProtocolData<ThrottleE5, true>();
  AddRecvProtocolData<SteeringE5, true>();
  AddRecvProtocolData<GearE5, true>();
  AddRecvProtocolData<MiscE5, true>();
  AddRecvProtocolData<WheelspeedE5, true>();
  AddRecvProtocolData<AccelE5, true>();
  AddRecvProtocolData<GyroE5, true>();
  AddRecvProtocolData<GpsE5, true>();
  AddRecvProtocolData<TirepressureE5, true>();
  AddRecvProtocolData<FuellevelE5, true>();
  AddRecvProtocolData<BrakeinfoE5, true>();
  AddRecvProtocolData<ThrottleinfoE5, true>();
  AddRecvProtocolData<VersionE5, true>();
}

BydMessageManager::~BydMessageManager() {}

}  // namespace byd
}  // namespace canbus
}  // namespace apollo
