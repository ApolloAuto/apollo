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

#include "modules/canbus_vehicle/ge3/proto/ge3.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ge3 {

class Pceps204 : public ::apollo::drivers::canbus::ProtocolData<
                     ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;

  Pceps204();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Steer speed request', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'name': 'PC_SteerSpdReq', 'is_signed_var':
  // False, 'physical_range': '[0|500]', 'bit': 31, 'type': 'int', 'order':
  // 'motorola', 'physical_unit': 'deg/s'}
  Pceps204* set_pc_steerspdreq(int pc_steerspdreq);

  // config detail: {'description': 'Steer control enable', 'enum': {0:
  // 'PC_STEERENABLE_DISABLE', 1: 'PC_STEERENABLE_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_SteerEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pceps204* set_pc_steerenable(Pc_eps_204::Pc_steerenableType pc_steerenable);

  // config detail: {'description': 'Steer angle request', 'offset': -500.0,
  // 'precision': 0.1, 'len': 16, 'name': 'PC_SteerAngReq', 'is_signed_var':
  // False, 'physical_range': '[-500|500]', 'bit': 15, 'type': 'double',
  // 'order': 'motorola', 'physical_unit': 'deg'}
  Pceps204* set_pc_steerangreq(double pc_steerangreq);

 private:
  // config detail: {'description': 'Steer speed request', 'offset': 0.0,
  // 'precision': 1.0, 'len': 16, 'name': 'PC_SteerSpdReq', 'is_signed_var':
  // False, 'physical_range': '[0|500]', 'bit': 31, 'type': 'int', 'order':
  // 'motorola', 'physical_unit': 'deg/s'}
  void set_p_pc_steerspdreq(uint8_t* data, int pc_steerspdreq);

  // config detail: {'description': 'Steer control enable', 'enum': {0:
  // 'PC_STEERENABLE_DISABLE', 1: 'PC_STEERENABLE_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_SteerEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_steerenable(uint8_t* data,
                            Pc_eps_204::Pc_steerenableType pc_steerenable);

  // config detail: {'description': 'Steer angle request', 'offset': -500.0,
  // 'precision': 0.1, 'len': 16, 'name': 'PC_SteerAngReq', 'is_signed_var':
  // False, 'physical_range': '[-500|500]', 'bit': 15, 'type': 'double',
  // 'order': 'motorola', 'physical_unit': 'deg'}
  void set_p_pc_steerangreq(uint8_t* data, double pc_steerangreq);

 private:
  int pc_steerspdreq_;
  Pc_eps_204::Pc_steerenableType pc_steerenable_;
  double pc_steerangreq_;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
