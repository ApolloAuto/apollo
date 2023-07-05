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

#include "modules/canbus_vehicle/wey/wey_message_manager.h"

#include "modules/canbus_vehicle/wey/protocol/ads1_111.h"
#include "modules/canbus_vehicle/wey/protocol/ads3_38e.h"
#include "modules/canbus_vehicle/wey/protocol/ads_eps_113.h"
#include "modules/canbus_vehicle/wey/protocol/ads_req_vin_390.h"
#include "modules/canbus_vehicle/wey/protocol/ads_shifter_115.h"

#include "modules/canbus_vehicle/wey/protocol/fail_241.h"
#include "modules/canbus_vehicle/wey/protocol/fbs1_243.h"
#include "modules/canbus_vehicle/wey/protocol/fbs2_240.h"
#include "modules/canbus_vehicle/wey/protocol/fbs3_237.h"
#include "modules/canbus_vehicle/wey/protocol/fbs4_235.h"
#include "modules/canbus_vehicle/wey/protocol/status_310.h"
#include "modules/canbus_vehicle/wey/protocol/vin_resp1_391.h"
#include "modules/canbus_vehicle/wey/protocol/vin_resp2_392.h"
#include "modules/canbus_vehicle/wey/protocol/vin_resp3_393.h"

namespace apollo {
namespace canbus {
namespace wey {

WeyMessageManager::WeyMessageManager() {
  // Control Messages
  AddSendProtocolData<Ads1111, true>();
  AddSendProtocolData<Ads338e, true>();
  AddSendProtocolData<Adseps113, true>();
  AddSendProtocolData<Adsreqvin390, true>();
  AddSendProtocolData<Adsshifter115, true>();

  // Report Messages
  AddRecvProtocolData<Fail241, true>();
  AddRecvProtocolData<Fbs1243, true>();
  AddRecvProtocolData<Fbs2240, true>();
  AddRecvProtocolData<Fbs3237, true>();
  AddRecvProtocolData<Fbs4235, true>();
  AddRecvProtocolData<Status310, true>();
  AddRecvProtocolData<Vinresp1391, true>();
  AddRecvProtocolData<Vinresp2392, true>();
  AddRecvProtocolData<Vinresp3393, true>();
}

WeyMessageManager::~WeyMessageManager() {}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
