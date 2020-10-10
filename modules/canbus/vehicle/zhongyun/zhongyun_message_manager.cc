/* Copyright 2019 The Apollo Authors. All Rights Reserved.

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

#include "modules/canbus/vehicle/zhongyun/zhongyun_message_manager.h"

#include "modules/canbus/vehicle/zhongyun/protocol/brake_control_a4.h"
#include "modules/canbus/vehicle/zhongyun/protocol/gear_control_a1.h"
#include "modules/canbus/vehicle/zhongyun/protocol/parking_control_a5.h"
#include "modules/canbus/vehicle/zhongyun/protocol/steering_control_a2.h"
#include "modules/canbus/vehicle/zhongyun/protocol/torque_control_a3.h"

#include "modules/canbus/vehicle/zhongyun/protocol/enable_state_feedback_c3.h"
#include "modules/canbus/vehicle/zhongyun/protocol/error_state_e1.h"
#include "modules/canbus/vehicle/zhongyun/protocol/vehicle_state_feedback_2_c4.h"
#include "modules/canbus/vehicle/zhongyun/protocol/vehicle_state_feedback_c1.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

ZhongyunMessageManager::ZhongyunMessageManager() {
  // Control Messages
  AddSendProtocolData<Brakecontrola4, true>();
  AddSendProtocolData<Gearcontrola1, true>();
  AddSendProtocolData<Parkingcontrola5, true>();
  AddSendProtocolData<Steeringcontrola2, true>();
  AddSendProtocolData<Torquecontrola3, true>();

  // Report Messages
  AddRecvProtocolData<Enablestatefeedbackc3, true>();
  AddRecvProtocolData<Errorstatee1, true>();
  AddRecvProtocolData<Vehiclestatefeedback2c4, true>();
  AddRecvProtocolData<Vehiclestatefeedbackc1, true>();
}

ZhongyunMessageManager::~ZhongyunMessageManager() {}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
