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

#include "modules/canbus/vehicle/%(car_type_lower)s/%(car_type_lower)s_message_manager.h"

%(control_header_list)s

%(report_header_list)s

namespace apollo {
namespace canbus {
namespace %(car_type_lower)s {

%(car_type_cap)sMessageManager::%(car_type_cap)sMessageManager() {
  // Control Messages
%(control_add_list)s

  // Report Messages
%(report_add_list)s
}

%(car_type_cap)sMessageManager::~%(car_type_cap)sMessageManager() {}

}  // namespace %(car_type_lower)s
}  // namespace canbus
}  // namespace apollo
