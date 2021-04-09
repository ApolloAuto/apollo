/* Copyright 2018 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/gem/gem_message_manager.h"

#include "modules/canbus/vehicle/gem/protocol/accel_cmd_67.h"
#include "modules/canbus/vehicle/gem/protocol/brake_cmd_6b.h"
#include "modules/canbus/vehicle/gem/protocol/global_cmd_69.h"
#include "modules/canbus/vehicle/gem/protocol/headlight_cmd_76.h"
#include "modules/canbus/vehicle/gem/protocol/horn_cmd_78.h"
#include "modules/canbus/vehicle/gem/protocol/shift_cmd_65.h"
#include "modules/canbus/vehicle/gem/protocol/steering_cmd_6d.h"
#include "modules/canbus/vehicle/gem/protocol/turn_cmd_63.h"
#include "modules/canbus/vehicle/gem/protocol/wiper_cmd_90.h"

#include "modules/canbus/vehicle/gem/protocol/accel_rpt_68.h"
#include "modules/canbus/vehicle/gem/protocol/brake_motor_rpt_1_70.h"
#include "modules/canbus/vehicle/gem/protocol/brake_motor_rpt_2_71.h"
#include "modules/canbus/vehicle/gem/protocol/brake_motor_rpt_3_72.h"
#include "modules/canbus/vehicle/gem/protocol/brake_rpt_6c.h"
#include "modules/canbus/vehicle/gem/protocol/date_time_rpt_83.h"
#include "modules/canbus/vehicle/gem/protocol/global_rpt_6a.h"
#include "modules/canbus/vehicle/gem/protocol/headlight_rpt_77.h"
#include "modules/canbus/vehicle/gem/protocol/horn_rpt_79.h"
#include "modules/canbus/vehicle/gem/protocol/lat_lon_heading_rpt_82.h"
#include "modules/canbus/vehicle/gem/protocol/parking_brake_status_rpt_80.h"
#include "modules/canbus/vehicle/gem/protocol/shift_rpt_66.h"
#include "modules/canbus/vehicle/gem/protocol/steering_motor_rpt_1_73.h"
#include "modules/canbus/vehicle/gem/protocol/steering_motor_rpt_2_74.h"
#include "modules/canbus/vehicle/gem/protocol/steering_motor_rpt_3_75.h"
#include "modules/canbus/vehicle/gem/protocol/steering_rpt_1_6e.h"
#include "modules/canbus/vehicle/gem/protocol/turn_rpt_64.h"
#include "modules/canbus/vehicle/gem/protocol/vehicle_speed_rpt_6f.h"
#include "modules/canbus/vehicle/gem/protocol/wheel_speed_rpt_7a.h"
#include "modules/canbus/vehicle/gem/protocol/wiper_rpt_91.h"
#include "modules/canbus/vehicle/gem/protocol/yaw_rate_rpt_81.h"

namespace apollo {
namespace canbus {
namespace gem {

GemMessageManager::GemMessageManager() {
  // Control Messages
  AddSendProtocolData<Accelcmd67, true>();
  AddSendProtocolData<Brakecmd6b, true>();
  AddSendProtocolData<Globalcmd69, true>();
  AddSendProtocolData<Headlightcmd76, true>();
  AddSendProtocolData<Horncmd78, true>();
  AddSendProtocolData<Shiftcmd65, true>();
  AddSendProtocolData<Steeringcmd6d, true>();
  AddSendProtocolData<Turncmd63, true>();
  AddSendProtocolData<Wipercmd90, true>();

  // Report Messages
  AddRecvProtocolData<Accelrpt68, true>();
  AddRecvProtocolData<Brakemotorrpt170, true>();
  AddRecvProtocolData<Brakemotorrpt271, true>();
  AddRecvProtocolData<Brakemotorrpt372, true>();
  AddRecvProtocolData<Brakerpt6c, true>();
  AddRecvProtocolData<Datetimerpt83, true>();
  AddRecvProtocolData<Globalrpt6a, true>();
  AddRecvProtocolData<Headlightrpt77, true>();
  AddRecvProtocolData<Hornrpt79, true>();
  AddRecvProtocolData<Latlonheadingrpt82, true>();
  AddRecvProtocolData<Parkingbrakestatusrpt80, true>();
  AddRecvProtocolData<Shiftrpt66, true>();
  AddRecvProtocolData<Steeringmotorrpt173, true>();
  AddRecvProtocolData<Steeringmotorrpt274, true>();
  AddRecvProtocolData<Steeringmotorrpt375, true>();
  AddRecvProtocolData<Steeringrpt16e, true>();
  AddRecvProtocolData<Turnrpt64, true>();
  AddRecvProtocolData<Vehiclespeedrpt6f, true>();
  AddRecvProtocolData<Wheelspeedrpt7a, true>();
  AddRecvProtocolData<Wiperrpt91, true>();
  AddRecvProtocolData<Yawraterpt81, true>();
}

GemMessageManager::~GemMessageManager() {}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
