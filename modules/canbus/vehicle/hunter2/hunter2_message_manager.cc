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

#include "modules/canbus/vehicle/hunter2/hunter2_message_manager.h"

#include "modules/canbus/vehicle/hunter2/protocol/brake_control_command_131.h"
#include "modules/canbus/vehicle/hunter2/protocol/control_mode_setting_421.h"
#include "modules/canbus/vehicle/hunter2/protocol/motion_control_instruction_111.h"
#include "modules/canbus/vehicle/hunter2/protocol/status_setting_441.h"

#include "modules/canbus/vehicle/hunter2/protocol/bms_data_feedback_361.h"
#include "modules/canbus/vehicle/hunter2/protocol/bms_status_feedback_362.h"
#include "modules/canbus/vehicle/hunter2/protocol/chassis_status_feedback_211.h"
#include "modules/canbus/vehicle/hunter2/protocol/high_leftback_motor_feedback_253.h"
#include "modules/canbus/vehicle/hunter2/protocol/high_rightback_motor_feedback_252.h"
#include "modules/canbus/vehicle/hunter2/protocol/high_steering_motor_feedback_251.h"
#include "modules/canbus/vehicle/hunter2/protocol/low_leftback_motor_feedback_263.h"
#include "modules/canbus/vehicle/hunter2/protocol/low_rightback_motor_feedback_262.h"
#include "modules/canbus/vehicle/hunter2/protocol/low_steering_motor_feedback_261.h"
#include "modules/canbus/vehicle/hunter2/protocol/mileage_feedback_311.h"
#include "modules/canbus/vehicle/hunter2/protocol/motion_control_feedback_221.h"
#include "modules/canbus/vehicle/hunter2/protocol/steering_zore_setting_431.h"
#include "modules/canbus/vehicle/hunter2/protocol/steering_zore_setting_feedback_43a.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

Hunter2MessageManager::Hunter2MessageManager() {
  // Control Messages
  AddSendProtocolData<Brakecontrolcommand131, true>();
  AddSendProtocolData<Controlmodesetting421, true>();
  AddSendProtocolData<Motioncontrolinstruction111, true>();
  AddSendProtocolData<Statussetting441, true>();

  // Report Messages
  AddRecvProtocolData<Bmsdatafeedback361, true>();
  AddRecvProtocolData<Bmsstatusfeedback362, true>();
  AddRecvProtocolData<Chassisstatusfeedback211, true>();
  AddRecvProtocolData<Highleftbackmotorfeedback253, true>();
  AddRecvProtocolData<Highrightbackmotorfeedback252, true>();
  AddRecvProtocolData<Highsteeringmotorfeedback251, true>();
  AddRecvProtocolData<Lowleftbackmotorfeedback263, true>();
  AddRecvProtocolData<Lowrightbackmotorfeedback262, true>();
  AddRecvProtocolData<Lowsteeringmotorfeedback261, true>();
  AddRecvProtocolData<Mileagefeedback311, true>();
  AddRecvProtocolData<Motioncontrolfeedback221, true>();
  AddRecvProtocolData<Steeringzoresetting431, true>();
  AddRecvProtocolData<Steeringzoresettingfeedback43a, true>();
}

Hunter2MessageManager::~Hunter2MessageManager() {}

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
