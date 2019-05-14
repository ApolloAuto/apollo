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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

namespace apollo {
namespace canbus {
namespace %(car_type_namespace)s {

using ::apollo::drivers::canbus::MessageManager;

class %(car_type_cap)sMessageManager
	: public MessageManager<::apollo::canbus::ChassisDetail> {
 public:
  %(car_type_cap)sMessageManager();
  virtual ~%(car_type_cap)sMessageManager();
};

}  // namespace %(car_type_namespace)s
}  // namespace canbus
}  // namespace apollo


