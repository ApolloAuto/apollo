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

#include "modules/canbus/vehicle/ge3/ge3_message_manager.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_bcm_201.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_bcs_202.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_epb_203.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_eps_204.h"
#include "modules/canbus/vehicle/ge3/protocol/pc_vcu_205.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_1_301.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_2_302.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_3_303.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcm_304.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_1_306.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_2_307.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_bcs_3_308.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_epb_310.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_eps_311.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_1_312.h"
#include "modules/canbus/vehicle/ge3/protocol/scu_vcu_2_313.h"

namespace apollo {
namespace canbus {
namespace ge3 {

Ge3MessageManager::Ge3MessageManager() {
  // Control Messages
  AddSendProtocolData<Pcbcm201, true>();
  AddSendProtocolData<Pcbcs202, true>();
  AddSendProtocolData<Pcepb203, true>();
  AddSendProtocolData<Pceps204, true>();
  AddSendProtocolData<Pcvcu205, true>();

  // Report Messages
  AddRecvProtocolData<Scu1301, true>();
  AddRecvProtocolData<Scu2302, true>();
  AddRecvProtocolData<Scu3303, true>();
  AddRecvProtocolData<Scubcm304, true>();
  AddRecvProtocolData<Scubcs1306, true>();
  AddRecvProtocolData<Scubcs2307, true>();
  AddRecvProtocolData<Scubcs3308, true>();
  AddRecvProtocolData<Scuepb310, true>();
  AddRecvProtocolData<Scueps311, true>();
  AddRecvProtocolData<Scuvcu1312, true>();
  AddRecvProtocolData<Scuvcu2313, true>();
}

Ge3MessageManager::~Ge3MessageManager() {}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
