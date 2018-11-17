/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/%(car_type_lower)s/protocol/%(protocol_name_lower)s.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace %(car_type_lower)s {

using ::apollo::drivers::canbus::Byte;

%(classname)s::%(classname)s() {}
const int32_t %(classname)s::ID = 0x%(id_upper)s;

void %(classname)s::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
%(set_var_to_protocol_list)s
}
%(func_impl_list)s
}  // namespace %(car_type_lower)s
}  // namespace canbus
}  // namespace apollo
