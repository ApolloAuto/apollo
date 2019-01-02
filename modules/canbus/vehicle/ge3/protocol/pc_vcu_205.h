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
namespace ge3 {

class Pcvcu205 : public ::apollo::drivers::canbus::ProtocolData<
                     ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Pcvcu205();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Acceleration pedal request', 'offset': 0.0,
  // 'precision': 0.05, 'len': 12, 'name': 'PC_AccPedReq', 'is_signed_var':
  // False, 'physical_range': '[0|100]', 'bit': 15, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': '%'}
  Pcvcu205* set_pc_accpedreq(double pc_accpedreq);

  // config detail: {'description': 'Acceleration pedal control enable', 'enum':
  // {0: 'PC_ACCPEDENABLE_DISABLE', 1: 'PC_ACCPEDENABLE_ENABLE'},
  // 'precision': 1.0, 'len': 1, 'name': 'PC_AccPedEnable', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Pcvcu205* set_pc_accpedenable(
      Pc_vcu_205::Pc_accpedenableType pc_accpedenable);

  // config detail: {'description': 'Torque request', 'offset': -3000.0,
  // 'precision': 1.5, 'len': 12, 'name': 'PC_TorqReq', 'is_signed_var': False,
  // 'physical_range': '[-3000|3000]', 'bit': 19, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'Nm'}
  Pcvcu205* set_pc_torqreq(double pc_torqreq);

  // config detail: {'description': 'Torque control enable', 'enum': {0:
  // 'PC_TORQENABLE_DISABLE', 1: 'PC_TORQENABLE_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_TorqEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcvcu205* set_pc_torqenable(Pc_vcu_205::Pc_torqenableType pc_torqenable);

  // config detail: {'description': 'Gear request', 'enum': {0:
  // 'PC_GEARREQ_INVALID', 1: 'PC_GEARREQ_DRIVE', 2: 'PC_GEARREQ_NEUTRAL', 3:
  // 'PC_GEARREQ_REVERSE', 4: 'PC_GEARREQ_PARK'}, 'precision': 1.0, 'len': 3,
  // 'name': 'PC_GearReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcvcu205* set_pc_gearreq(Pc_vcu_205::Pc_gearreqType pc_gearreq);

  // config detail: {'description': 'Gear control enable', 'enum': {0:
  // 'PC_GEARENABLE_DISABLE', 1: 'PC_GEARENABLE_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_GearEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcvcu205* set_pc_gearenable(Pc_vcu_205::Pc_gearenableType pc_gearenable);

 private:
  // config detail: {'description': 'Acceleration pedal request', 'offset': 0.0,
  // 'precision': 0.05, 'len': 12, 'name': 'PC_AccPedReq', 'is_signed_var':
  // False, 'physical_range': '[0|100]', 'bit': 15, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': '%'}
  void set_p_pc_accpedreq(uint8_t* data, double pc_accpedreq);

  // config detail: {'description': 'Acceleration pedal control enable', 'enum':
  // {0: 'PC_ACCPEDENABLE_DISABLE', 1: 'PC_ACCPEDENABLE_ENABLE'},
  // 'precision': 1.0, 'len': 1, 'name': 'PC_AccPedEnable', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_pc_accpedenable(uint8_t* data,
                             Pc_vcu_205::Pc_accpedenableType pc_accpedenable);

  // config detail: {'description': 'Torque request', 'offset': -3000.0,
  // 'precision': 1.5, 'len': 12, 'name': 'PC_TorqReq', 'is_signed_var': False,
  // 'physical_range': '[-3000|3000]', 'bit': 19, 'type': 'double', 'order':
  // 'motorola', 'physical_unit': 'Nm'}
  void set_p_pc_torqreq(uint8_t* data, double pc_torqreq);

  // config detail: {'description': 'Torque control enable', 'enum': {0:
  // 'PC_TORQENABLE_DISABLE', 1: 'PC_TORQENABLE_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_TorqEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_torqenable(uint8_t* data,
                           Pc_vcu_205::Pc_torqenableType pc_torqenable);

  // config detail: {'description': 'Gear request', 'enum': {0:
  // 'PC_GEARREQ_INVALID', 1: 'PC_GEARREQ_DRIVE', 2: 'PC_GEARREQ_NEUTRAL', 3:
  // 'PC_GEARREQ_REVERSE', 4: 'PC_GEARREQ_PARK'}, 'precision': 1.0, 'len': 3,
  // 'name': 'PC_GearReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|7]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_gearreq(uint8_t* data, Pc_vcu_205::Pc_gearreqType pc_gearreq);

  // config detail: {'description': 'Gear control enable', 'enum': {0:
  // 'PC_GEARENABLE_DISABLE', 1: 'PC_GEARENABLE_ENABLE'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_GearEnable', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_gearenable(uint8_t* data,
                           Pc_vcu_205::Pc_gearenableType pc_gearenable);

 private:
  double pc_accpedreq_;
  Pc_vcu_205::Pc_accpedenableType pc_accpedenable_;
  double pc_torqreq_;
  Pc_vcu_205::Pc_torqenableType pc_torqenable_;
  Pc_vcu_205::Pc_gearreqType pc_gearreq_;
  Pc_vcu_205::Pc_gearenableType pc_gearenable_;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
