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

#ifndef MODULES_CANBUS_VEHICLE_%(car_type_upper)s_PROTOCOL_%(protocol_name_upper)s_H_
#define MODULES_CANBUS_VEHICLE_%(car_type_upper)s_PROTOCOL_%(protocol_name_upper)s_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace %(car_type_lower)s {

class %(classname)s : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  %(classname)s();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:
%(func_declare_list)s
};

}  // namespace %(car_type_lower)s
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_%(car_type_upper)s_PROTOCOL_%(protocol_name_upper)s_H_
