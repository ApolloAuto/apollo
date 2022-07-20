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

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Vcubreaksyscmd18ff85a7 : public ::apollo::drivers::canbus::ProtocolData<
                                   ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vcubreaksyscmd18ff85a7();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  bool vcu_brk_autoparking_request(const std::uint8_t* bytes,
                                   const int32_t length) const;

  bool vcu_brk_initivate_enable(const std::uint8_t* bytes,
                                const int32_t length) const;

  double vcu_brk_right_pressure(const std::uint8_t* bytes,
                                const int32_t length) const;

  double vcu_brk_left_pressure(const std::uint8_t* bytes,
                               const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
