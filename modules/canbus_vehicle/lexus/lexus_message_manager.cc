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

#include "modules/canbus_vehicle/lexus/lexus_message_manager.h"

#include "modules/canbus_vehicle/lexus/protocol/accel_cmd_100.h"
#include "modules/canbus_vehicle/lexus/protocol/brake_cmd_104.h"
#include "modules/canbus_vehicle/lexus/protocol/cruise_control_buttons_cmd_108.h"
#include "modules/canbus_vehicle/lexus/protocol/dash_controls_right_rpt_210.h"
#include "modules/canbus_vehicle/lexus/protocol/hazard_lights_cmd_114.h"
#include "modules/canbus_vehicle/lexus/protocol/headlight_cmd_118.h"
#include "modules/canbus_vehicle/lexus/protocol/horn_cmd_11c.h"
#include "modules/canbus_vehicle/lexus/protocol/parking_brake_cmd_124.h"
#include "modules/canbus_vehicle/lexus/protocol/shift_cmd_128.h"
#include "modules/canbus_vehicle/lexus/protocol/steering_cmd_12c.h"
#include "modules/canbus_vehicle/lexus/protocol/turn_cmd_130.h"
#include "modules/canbus_vehicle/lexus/protocol/wiper_cmd_134.h"

#include "modules/canbus_vehicle/lexus/protocol/accel_aux_rpt_300.h"
#include "modules/canbus_vehicle/lexus/protocol/accel_rpt_200.h"
#include "modules/canbus_vehicle/lexus/protocol/brake_aux_rpt_304.h"
#include "modules/canbus_vehicle/lexus/protocol/brake_motor_rpt_1_401.h"
#include "modules/canbus_vehicle/lexus/protocol/brake_motor_rpt_2_402.h"
#include "modules/canbus_vehicle/lexus/protocol/brake_motor_rpt_3_403.h"
#include "modules/canbus_vehicle/lexus/protocol/brake_rpt_204.h"
#include "modules/canbus_vehicle/lexus/protocol/component_rpt_20.h"
#include "modules/canbus_vehicle/lexus/protocol/cruise_control_buttons_rpt_208.h"
#include "modules/canbus_vehicle/lexus/protocol/dash_controls_left_cmd_10c.h"
#include "modules/canbus_vehicle/lexus/protocol/dash_controls_left_rpt_20c.h"
#include "modules/canbus_vehicle/lexus/protocol/dash_controls_right_cmd_110.h"
#include "modules/canbus_vehicle/lexus/protocol/date_time_rpt_40f.h"
#include "modules/canbus_vehicle/lexus/protocol/detected_object_rpt_411.h"
#include "modules/canbus_vehicle/lexus/protocol/door_rpt_417.h"
#include "modules/canbus_vehicle/lexus/protocol/global_rpt_10.h"
#include "modules/canbus_vehicle/lexus/protocol/hazard_lights_rpt_214.h"
#include "modules/canbus_vehicle/lexus/protocol/headlight_aux_rpt_318.h"
#include "modules/canbus_vehicle/lexus/protocol/headlight_rpt_218.h"
#include "modules/canbus_vehicle/lexus/protocol/horn_rpt_21c.h"
#include "modules/canbus_vehicle/lexus/protocol/interior_lights_rpt_416.h"
#include "modules/canbus_vehicle/lexus/protocol/lat_lon_heading_rpt_40e.h"
#include "modules/canbus_vehicle/lexus/protocol/media_controls_cmd_120.h"
#include "modules/canbus_vehicle/lexus/protocol/media_controls_rpt_220.h"
#include "modules/canbus_vehicle/lexus/protocol/occupancy_rpt_415.h"
#include "modules/canbus_vehicle/lexus/protocol/parking_brake_rpt_224.h"
#include "modules/canbus_vehicle/lexus/protocol/rear_lights_rpt_418.h"
#include "modules/canbus_vehicle/lexus/protocol/shift_aux_rpt_328.h"
#include "modules/canbus_vehicle/lexus/protocol/shift_rpt_228.h"
#include "modules/canbus_vehicle/lexus/protocol/steering_aux_rpt_32c.h"
#include "modules/canbus_vehicle/lexus/protocol/steering_motor_rpt_1_404.h"
#include "modules/canbus_vehicle/lexus/protocol/steering_motor_rpt_2_405.h"
#include "modules/canbus_vehicle/lexus/protocol/steering_motor_rpt_3_406.h"
#include "modules/canbus_vehicle/lexus/protocol/steering_rpt_22c.h"
#include "modules/canbus_vehicle/lexus/protocol/turn_aux_rpt_330.h"
#include "modules/canbus_vehicle/lexus/protocol/turn_rpt_230.h"
#include "modules/canbus_vehicle/lexus/protocol/veh_dynamics_rpt_413.h"
#include "modules/canbus_vehicle/lexus/protocol/veh_specific_rpt_1_412.h"
#include "modules/canbus_vehicle/lexus/protocol/vehicle_speed_rpt_400.h"
#include "modules/canbus_vehicle/lexus/protocol/vin_rpt_414.h"
#include "modules/canbus_vehicle/lexus/protocol/wheel_speed_rpt_407.h"
#include "modules/canbus_vehicle/lexus/protocol/wiper_aux_rpt_334.h"
#include "modules/canbus_vehicle/lexus/protocol/wiper_rpt_234.h"
#include "modules/canbus_vehicle/lexus/protocol/yaw_rate_rpt_40d.h"

namespace apollo {
namespace canbus {
namespace lexus {

LexusMessageManager::LexusMessageManager() {
  // Control Messages
  AddSendProtocolData<Accelcmd100, true>();
  AddSendProtocolData<Brakecmd104, true>();
  AddSendProtocolData<Cruisecontrolbuttonscmd108, true>();
  AddSendProtocolData<Dashcontrolsrightrpt210, true>();
  AddSendProtocolData<Hazardlightscmd114, true>();
  AddSendProtocolData<Headlightcmd118, true>();
  AddSendProtocolData<Horncmd11c, true>();
  AddSendProtocolData<Parkingbrakecmd124, true>();
  AddSendProtocolData<Shiftcmd128, true>();
  AddSendProtocolData<Steeringcmd12c, true>();
  AddSendProtocolData<Turncmd130, true>();
  AddSendProtocolData<Wipercmd134, true>();

  // Report Messages
  AddRecvProtocolData<Accelauxrpt300, true>();
  AddRecvProtocolData<Accelrpt200, true>();
  AddRecvProtocolData<Brakeauxrpt304, true>();
  AddRecvProtocolData<Brakemotorrpt1401, true>();
  AddRecvProtocolData<Brakemotorrpt2402, true>();
  AddRecvProtocolData<Brakemotorrpt3403, true>();
  AddRecvProtocolData<Brakerpt204, true>();
  AddRecvProtocolData<Componentrpt20, true>();
  AddRecvProtocolData<Cruisecontrolbuttonsrpt208, true>();
  AddRecvProtocolData<Dashcontrolsleftcmd10c, true>();
  AddRecvProtocolData<Dashcontrolsleftrpt20c, true>();
  AddRecvProtocolData<Dashcontrolsrightcmd110, true>();
  AddRecvProtocolData<Datetimerpt40f, true>();
  AddRecvProtocolData<Detectedobjectrpt411, true>();
  AddRecvProtocolData<Doorrpt417, true>();
  AddRecvProtocolData<Globalrpt10, true>();
  AddRecvProtocolData<Hazardlightsrpt214, true>();
  AddRecvProtocolData<Headlightauxrpt318, true>();
  AddRecvProtocolData<Headlightrpt218, true>();
  AddRecvProtocolData<Hornrpt21c, true>();
  AddRecvProtocolData<Interiorlightsrpt416, true>();
  AddRecvProtocolData<Latlonheadingrpt40e, true>();
  AddRecvProtocolData<Mediacontrolscmd120, true>();
  AddRecvProtocolData<Mediacontrolsrpt220, true>();
  AddRecvProtocolData<Occupancyrpt415, true>();
  AddRecvProtocolData<Parkingbrakerpt224, true>();
  AddRecvProtocolData<Rearlightsrpt418, true>();
  AddRecvProtocolData<Shiftauxrpt328, true>();
  AddRecvProtocolData<Shiftrpt228, true>();
  AddRecvProtocolData<Steeringauxrpt32c, true>();
  AddRecvProtocolData<Steeringmotorrpt1404, true>();
  AddRecvProtocolData<Steeringmotorrpt2405, true>();
  AddRecvProtocolData<Steeringmotorrpt3406, true>();
  AddRecvProtocolData<Steeringrpt22c, true>();
  AddRecvProtocolData<Turnauxrpt330, true>();
  AddRecvProtocolData<Turnrpt230, true>();
  AddRecvProtocolData<Vehdynamicsrpt413, true>();
  AddRecvProtocolData<Vehiclespeedrpt400, true>();
  AddRecvProtocolData<Vehspecificrpt1412, true>();
  AddRecvProtocolData<Vinrpt414, true>();
  AddRecvProtocolData<Wheelspeedrpt407, true>();
  AddRecvProtocolData<Wiperauxrpt334, true>();
  AddRecvProtocolData<Wiperrpt234, true>();
  AddRecvProtocolData<Yawraterpt40d, true>();
}

LexusMessageManager::~LexusMessageManager() {}

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
