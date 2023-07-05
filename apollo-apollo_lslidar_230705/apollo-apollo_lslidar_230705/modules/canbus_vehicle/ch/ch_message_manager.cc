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

#include "modules/canbus_vehicle/ch/ch_message_manager.h"

#include "modules/canbus_vehicle/ch/protocol/throttle_command_110.h"
#include "modules/canbus_vehicle/ch/protocol/brake_command_111.h"
#include "modules/canbus_vehicle/ch/protocol/steer_command_112.h"
#include "modules/canbus_vehicle/ch/protocol/turnsignal_command_113.h"
#include "modules/canbus_vehicle/ch/protocol/gear_command_114.h"
#include "modules/canbus_vehicle/ch/protocol/vehicle_mode_command_116.h"

#include "modules/canbus_vehicle/ch/protocol/throttle_status__510.h"
#include "modules/canbus_vehicle/ch/protocol/brake_status__511.h"
#include "modules/canbus_vehicle/ch/protocol/steer_status__512.h"
#include "modules/canbus_vehicle/ch/protocol/turnsignal_status__513.h"
#include "modules/canbus_vehicle/ch/protocol/gear_status_514.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_1_515.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_2_516.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_3_517.h"
#include "modules/canbus_vehicle/ch/protocol/ecu_status_4_518.h"
#include "modules/canbus_vehicle/ch/protocol/vin_resp1_51b.h"
#include "modules/canbus_vehicle/ch/protocol/vin_resp2_51c.h"
#include "modules/canbus_vehicle/ch/protocol/vin_resp3_51d.h"
#include "modules/canbus_vehicle/ch/protocol/wheelspeed_report_51e.h"

namespace apollo {
namespace canbus {
namespace ch {

ChMessageManager::ChMessageManager() {
  // Control Messages
  AddSendProtocolData<Brakecommand111, true>();
  AddSendProtocolData<Gearcommand114, true>();
  AddSendProtocolData<Steercommand112, true>();
  AddSendProtocolData<Throttlecommand110, true>();
  AddSendProtocolData<Turnsignalcommand113, true>();
  AddSendProtocolData<Vehiclemodecommand116, true>();

  // Report Messages
  AddRecvProtocolData<Brakestatus511, true>();
  AddRecvProtocolData<Ecustatus1515, true>();
  AddRecvProtocolData<Ecustatus2516, true>();
  AddRecvProtocolData<Ecustatus3517, true>();
  AddRecvProtocolData<Ecustatus4518, true>();
  AddRecvProtocolData<Gearstatus514, true>();
  AddRecvProtocolData<Steerstatus512, true>();
  AddRecvProtocolData<Throttlestatus510, true>();
  AddRecvProtocolData<Turnsignalstatus513, true>();
  AddRecvProtocolData<Vinresp151b, true>();
  AddRecvProtocolData<Vinresp251c, true>();
  AddRecvProtocolData<Vinresp351d, true>();
  AddRecvProtocolData<Wheelspeedreport51e, true>();
}

ChMessageManager::~ChMessageManager() {}

}  // namespace ch
}  // namespace canbus
}  // namespace apollo
