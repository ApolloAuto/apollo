/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/neolix_edu/neolix_edu_message_manager.h"

#include "modules/canbus_vehicle/neolix_edu/protocol/ads_brake_command_46.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/ads_diagnosis_628.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/ads_drive_command_50.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/ads_eps_command_56.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/ads_light_horn_command_310.h"

#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_diagnosis1_626.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_diagresp_718.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_frontwheelspeed_353.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_rearwheelspeed_354.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_systemstate_11.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/aeb_wheelimpulse_355.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/pas_1st_data_311.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/pas_2nd_data_312.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_brake_report_47.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_drive_report_52.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_eps_report_57.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_nm_401.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_powerstatus_214.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_vehicle_fault_response_201.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_vehicle_info_response_502.h"
#include "modules/canbus_vehicle/neolix_edu/protocol/vcu_vehicle_status_report_101.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

Neolix_eduMessageManager::Neolix_eduMessageManager() {
  // Control Messages
  AddSendProtocolData<Adsbrakecommand46, true>();
  AddSendProtocolData<Adsdiagnosis628, true>();
  AddSendProtocolData<Adsdrivecommand50, true>();
  AddSendProtocolData<Adsepscommand56, true>();
  AddSendProtocolData<Adslighthorncommand310, true>();

  // Report Messages
  AddRecvProtocolData<Aebsystemstate11, true>();
  AddRecvProtocolData<Vcubrakereport47, true>();
  AddRecvProtocolData<Vcudrivereport52, true>();
  AddRecvProtocolData<Vcuepsreport57, true>();
  AddRecvProtocolData<Vcunm401, true>();
  AddRecvProtocolData<Vcupowerstatus214, true>();
  AddRecvProtocolData<Vcuvehiclefaultresponse201, true>();
  AddRecvProtocolData<Vcuvehicleinforesponse502, true>();
  AddRecvProtocolData<Vcuvehiclestatusreport101, true>();
  AddRecvProtocolData<Aebdiagnosis1626, true>();
  AddRecvProtocolData<Aebdiagresp718, true>();
  AddRecvProtocolData<Aebfrontwheelspeed353, true>();
  AddRecvProtocolData<Aebrearwheelspeed354, true>();
  AddRecvProtocolData<Aebwheelimpulse355, true>();
  AddRecvProtocolData<Pas1stdata311, true>();
  AddRecvProtocolData<Pas2nddata312, true>();
}

Neolix_eduMessageManager::~Neolix_eduMessageManager() {}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
