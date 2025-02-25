// Copyright 2025 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/canbus_vehicle/ros/ros_message_manager.h"

#include "modules/canbus_vehicle/ros/protocol/acc_fb.h"
#include "modules/canbus_vehicle/ros/protocol/ang_vel_fb.h"
#include "modules/canbus_vehicle/ros/protocol/twist_cmd.h"
#include "modules/canbus_vehicle/ros/protocol/twist_fb.h"

namespace apollo {
namespace canbus {
namespace ros {

RosMessageManager::RosMessageManager() {
  // Control Messages
  AddSendProtocolData<TwistCmd, true>();

  // Report Messages
  AddRecvProtocolData<TwistFb, true>();
  AddRecvProtocolData<AccFb, true>();
  AddRecvProtocolData<AngVelFb, true>();
}

RosMessageManager::~RosMessageManager() {}

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
