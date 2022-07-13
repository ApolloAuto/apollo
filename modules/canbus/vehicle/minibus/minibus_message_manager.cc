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

#include "modules/canbus/vehicle/minibus/minibus_message_manager.h"

#include "modules/canbus/vehicle/minibus/protocol/controller_parking_18ff8ca9.h"
#include "modules/canbus/vehicle/minibus/protocol/controller_pedal_cmd_18ff84a9.h"
#include "modules/canbus/vehicle/minibus/protocol/controller_status_requset_18ff86a9.h"
#include "modules/canbus/vehicle/minibus/protocol/controller_steering_cmd_18ff82a9.h"

#include "modules/canbus/vehicle/minibus/protocol/brake_nboost_ctrl_feedback_18ff9197.h"
#include "modules/canbus/vehicle/minibus/protocol/breaksystem_feedback_18ff87ab.h"
#include "modules/canbus/vehicle/minibus/protocol/bus_battery_meg_18fa1017.h"
#include "modules/canbus/vehicle/minibus/protocol/bus_inteligent_control_status_18fa0117.h"
#include "modules/canbus/vehicle/minibus/protocol/bus_mileage_18fee017.h"
#include "modules/canbus/vehicle/minibus/protocol/bus_vehicle_speed_msg_cfe6c17.h"
#include "modules/canbus/vehicle/minibus/protocol/bus_vehicle_status_18fa0517.h"
#include "modules/canbus/vehicle/minibus/protocol/eps_feedback_18ff83aa.h"
#include "modules/canbus/vehicle/minibus/protocol/parkingmode_feedback_18ff8dac.h"
#include "modules/canbus/vehicle/minibus/protocol/soc_18ffeb97.h"
#include "modules/canbus/vehicle/minibus/protocol/vcu_basic_message_18ffea97.h"
#include "modules/canbus/vehicle/minibus/protocol/vcu_breaksys_cmd_18ff85a7.h"
#include "modules/canbus/vehicle/minibus/protocol/vcu_drive_feedback_18ff7097.h"

namespace apollo {
namespace canbus {
namespace minibus {

MinibusMessageManager::MinibusMessageManager() {
  // Control Messages
  AddSendProtocolData<Controllerparking18ff8ca9, true>();
  AddSendProtocolData<Controllerpedalcmd18ff84a9, true>();
  AddSendProtocolData<Controllerstatusrequset18ff86a9, true>();
  AddSendProtocolData<Controllersteeringcmd18ff82a9, true>();

  // Report Messages
  AddRecvProtocolData<Brakenboostctrlfeedback18ff9197, true>();
  AddRecvProtocolData<Breaksystemfeedback18ff87ab, true>();
  AddRecvProtocolData<Busbatterymeg18fa1017, true>();
  AddRecvProtocolData<Businteligentcontrolstatus18fa0117, true>();
  AddRecvProtocolData<Busmileage18fee017, true>();
  AddRecvProtocolData<Busvehiclespeedmsgcfe6c17, true>();
  AddRecvProtocolData<Busvehiclestatus18fa0517, true>();
  AddRecvProtocolData<Epsfeedback18ff83aa, true>();
  AddRecvProtocolData<Parkingmodefeedback18ff8dac, true>();
  AddRecvProtocolData<Soc18ffeb97, true>();
  AddRecvProtocolData<Vcubasicmessage18ffea97, true>();
  AddRecvProtocolData<Vcubreaksyscmd18ff85a7, true>();
  AddRecvProtocolData<Vcudrivefeedback18ff7097, true>();
}

MinibusMessageManager::~MinibusMessageManager() {}

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
