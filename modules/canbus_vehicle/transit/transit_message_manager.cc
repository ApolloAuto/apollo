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

#include "modules/canbus_vehicle/transit/transit_message_manager.h"

#include "modules/canbus_vehicle/transit/protocol/adc_auxiliarycontrol_110.h"
#include "modules/canbus_vehicle/transit/protocol/adc_motioncontrol1_10.h"
#include "modules/canbus_vehicle/transit/protocol/adc_motioncontrollimits1_12.h"
#include "modules/canbus_vehicle/transit/protocol/llc_diag_brakecontrol_721.h"
#include "modules/canbus_vehicle/transit/protocol/llc_diag_steeringcontrol_722.h"

#include "modules/canbus_vehicle/transit/protocol/llc_auxiliaryfeedback_120.h"
#include "modules/canbus_vehicle/transit/protocol/llc_diag_fault_620.h"
#include "modules/canbus_vehicle/transit/protocol/llc_motioncommandfeedback1_22.h"
#include "modules/canbus_vehicle/transit/protocol/llc_motionfeedback1_20.h"
#include "modules/canbus_vehicle/transit/protocol/llc_motionfeedback2_21.h"
#include "modules/canbus_vehicle/transit/protocol/llc_vehiclelimits_24.h"
#include "modules/canbus_vehicle/transit/protocol/llc_vehiclestatus_25.h"

namespace apollo {
namespace canbus {
namespace transit {

TransitMessageManager::TransitMessageManager() {
  // Control Messages
  AddSendProtocolData<Adcauxiliarycontrol110, true>();
  AddSendProtocolData<Adcmotioncontrol110, true>();
  AddSendProtocolData<Adcmotioncontrollimits112, true>();
  AddSendProtocolData<Llcdiagbrakecontrol721, true>();
  AddSendProtocolData<Llcdiagsteeringcontrol722, true>();

  // Report Messages
  AddRecvProtocolData<Llcauxiliaryfeedback120, true>();
  AddRecvProtocolData<Llcdiagfault620, true>();
  AddRecvProtocolData<Llcmotioncommandfeedback122, true>();
  AddRecvProtocolData<Llcmotionfeedback120, true>();
  AddRecvProtocolData<Llcmotionfeedback221, true>();
  AddRecvProtocolData<Llcvehiclelimits24, true>();
  AddRecvProtocolData<Llcvehiclestatus25, true>();
}

TransitMessageManager::~TransitMessageManager() {}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
