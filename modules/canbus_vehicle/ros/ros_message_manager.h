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

#pragma once

#include "modules/canbus_vehicle/ros/proto/ros.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

namespace apollo {
namespace canbus {
namespace ros {

using ::apollo::drivers::canbus::MessageManager;

class RosMessageManager
    : public MessageManager<::apollo::canbus::Ros> {
 public:
  RosMessageManager();
  virtual ~RosMessageManager();
};

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
