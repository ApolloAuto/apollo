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

#include "modules/canbus/vehicle/%(car_type_lower)s/protocol/%(protocol_name_lower)s.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace %(car_type_lower)s {

using ::apollo::drivers::canbus::Byte;

const int32_t %(classname)s::ID = 0x%(id_upper)s;

// public
%(classname)s::%(classname)s() { Reset(); }

uint32_t %(classname)s::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void %(classname)s::UpdateData(uint8_t* data) {
%(set_private_var_list)s
}

void %(classname)s::Reset() {
  // TODO(All) :  you should check this manually
%(set_private_var_init_list)s
}
%(set_func_impl_list)s
}  // namespace %(car_type_lower)s
}  // namespace canbus
}  // namespace apollo
