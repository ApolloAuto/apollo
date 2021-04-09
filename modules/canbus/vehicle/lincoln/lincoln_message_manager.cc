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

#include "modules/canbus/vehicle/lincoln/lincoln_message_manager.h"

#include "modules/canbus/vehicle/lincoln/protocol/accel_6b.h"
#include "modules/canbus/vehicle/lincoln/protocol/brake_60.h"
#include "modules/canbus/vehicle/lincoln/protocol/brake_61.h"
#include "modules/canbus/vehicle/lincoln/protocol/brakeinfo_74.h"
#include "modules/canbus/vehicle/lincoln/protocol/fuellevel_72.h"
#include "modules/canbus/vehicle/lincoln/protocol/gear_66.h"
#include "modules/canbus/vehicle/lincoln/protocol/gear_67.h"
#include "modules/canbus/vehicle/lincoln/protocol/gps_6d.h"
#include "modules/canbus/vehicle/lincoln/protocol/gps_6e.h"
#include "modules/canbus/vehicle/lincoln/protocol/gps_6f.h"
#include "modules/canbus/vehicle/lincoln/protocol/gyro_6c.h"
#include "modules/canbus/vehicle/lincoln/protocol/license_7e.h"
#include "modules/canbus/vehicle/lincoln/protocol/misc_69.h"
#include "modules/canbus/vehicle/lincoln/protocol/steering_64.h"
#include "modules/canbus/vehicle/lincoln/protocol/steering_65.h"
#include "modules/canbus/vehicle/lincoln/protocol/surround_73.h"
#include "modules/canbus/vehicle/lincoln/protocol/throttle_62.h"
#include "modules/canbus/vehicle/lincoln/protocol/throttle_63.h"
#include "modules/canbus/vehicle/lincoln/protocol/throttleinfo_75.h"
#include "modules/canbus/vehicle/lincoln/protocol/tirepressure_71.h"
#include "modules/canbus/vehicle/lincoln/protocol/turnsignal_68.h"
#include "modules/canbus/vehicle/lincoln/protocol/version_7f.h"
#include "modules/canbus/vehicle/lincoln/protocol/wheelspeed_6a.h"

namespace apollo {
namespace canbus {
namespace lincoln {

LincolnMessageManager::LincolnMessageManager() {
  // TODO(Authors): verify which one is recv/sent
  AddSendProtocolData<Brake60, true>();
  AddSendProtocolData<Throttle62, true>();
  AddSendProtocolData<Steering64, true>();
  AddSendProtocolData<Gear66, true>();
  AddSendProtocolData<Turnsignal68, true>();

  AddRecvProtocolData<Brake61, true>();
  AddRecvProtocolData<Throttle63, true>();
  AddRecvProtocolData<Steering65, true>();
  AddRecvProtocolData<Gear67, true>();
  AddRecvProtocolData<Misc69, true>();
  AddRecvProtocolData<Wheelspeed6a, true>();
  AddRecvProtocolData<Accel6b, true>();
  AddRecvProtocolData<Gyro6c, true>();
  AddRecvProtocolData<Gps6d, true>();
  AddRecvProtocolData<Gps6e, true>();
  AddRecvProtocolData<Gps6f, true>();
  AddRecvProtocolData<Tirepressure71, true>();
  AddRecvProtocolData<Fuellevel72, true>();
  AddRecvProtocolData<Surround73, true>();
  AddRecvProtocolData<Brakeinfo74, true>();
  AddRecvProtocolData<Throttleinfo75, true>();
  AddRecvProtocolData<Version7f, true>();
  AddRecvProtocolData<License7e, true>();
}

LincolnMessageManager::~LincolnMessageManager() {}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
