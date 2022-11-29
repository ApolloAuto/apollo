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

class Pcbcm201 : public ::apollo::drivers::canbus::ProtocolData<
                     ::apollo::canbus::Ge3> {
 public:
  static const int32_t ID;

  Pcbcm201();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_REVERSELAMPREQ_NOREQ', 1: 'PC_REVERSELAMPREQ_REQ'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_ReverseLampReq', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  Pcbcm201* set_pc_reverselampreq(
      Pc_bcm_201::Pc_reverselampreqType pc_reverselampreq);

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_LOWBEAMREQ_NOREQ', 1: 'PC_LOWBEAMREQ_REQ'}, 'precision': 1.0, 'len': 1,
  // 'name': 'PC_LowBeamReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcbcm201* set_pc_lowbeamreq(Pc_bcm_201::Pc_lowbeamreqType pc_lowbeamreq);

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_HIGHBEAMREQ_NOREQ', 1: 'PC_HIGHBEAMREQ_REQ'}, 'precision': 1.0, 'len':
  // 1, 'name': 'PC_HighBeamReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcbcm201* set_pc_highbeamreq(Pc_bcm_201::Pc_highbeamreqType pc_highbeamreq);

  // config detail: {'description': 'Right turn lamp request', 'enum': {0:
  // 'PC_RIGHTTURNLAMPREQ_NOREQ', 1: 'PC_RIGHTTURNLAMPREQ_REQ'},
  // 'precision': 1.0, 'len': 1, 'name': 'PC_RightTurnLampReq', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  Pcbcm201* set_pc_rightturnlampreq(
      Pc_bcm_201::Pc_rightturnlampreqType pc_rightturnlampreq);

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_LEFTTURNLAMPREQ_NOREQ', 1: 'PC_LEFTTURNLAMPREQ_REQ'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_LeftTurnLampReq', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  Pcbcm201* set_pc_leftturnlampreq(
      Pc_bcm_201::Pc_leftturnlampreqType pc_leftturnlampreq);

  // config detail: {'description': 'Horn request', 'enum': {0:
  // 'PC_HORNREQ_NOREQ', 1: 'PC_HORNREQ_REQ'}, 'precision': 1.0, 'len': 1,
  // 'name': 'PC_HornReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  Pcbcm201* set_pc_hornreq(Pc_bcm_201::Pc_hornreqType pc_hornreq);

  // config detail: {'description': 'Hazard lamp request', 'enum': {0:
  // 'PC_HAZARDLAMPREQ_NOREQ', 1: 'PC_HAZARDLAMPREQ_REQ'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_HazardLampReq', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  Pcbcm201* set_pc_hazardlampreq(
      Pc_bcm_201::Pc_hazardlampreqType pc_hazardlampreq);

 private:
  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_REVERSELAMPREQ_NOREQ', 1: 'PC_REVERSELAMPREQ_REQ'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_ReverseLampReq', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  void set_p_pc_reverselampreq(
      uint8_t* data, Pc_bcm_201::Pc_reverselampreqType pc_reverselampreq);

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_LOWBEAMREQ_NOREQ', 1: 'PC_LOWBEAMREQ_REQ'}, 'precision': 1.0, 'len': 1,
  // 'name': 'PC_LowBeamReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_lowbeamreq(uint8_t* data,
                           Pc_bcm_201::Pc_lowbeamreqType pc_lowbeamreq);

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_HIGHBEAMREQ_NOREQ', 1: 'PC_HIGHBEAMREQ_REQ'}, 'precision': 1.0, 'len':
  // 1, 'name': 'PC_HighBeamReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_highbeamreq(uint8_t* data,
                            Pc_bcm_201::Pc_highbeamreqType pc_highbeamreq);

  // config detail: {'description': 'Right turn lamp request', 'enum': {0:
  // 'PC_RIGHTTURNLAMPREQ_NOREQ', 1: 'PC_RIGHTTURNLAMPREQ_REQ'},
  // 'precision': 1.0, 'len': 1, 'name': 'PC_RightTurnLampReq', 'is_signed_var':
  // False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum',
  // 'order': 'motorola', 'physical_unit': ''}
  void set_p_pc_rightturnlampreq(
      uint8_t* data, Pc_bcm_201::Pc_rightturnlampreqType pc_rightturnlampreq);

  // config detail: {'description': 'Left turn lamp request', 'enum': {0:
  // 'PC_LEFTTURNLAMPREQ_NOREQ', 1: 'PC_LEFTTURNLAMPREQ_REQ'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_LeftTurnLampReq', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  void set_p_pc_leftturnlampreq(
      uint8_t* data, Pc_bcm_201::Pc_leftturnlampreqType pc_leftturnlampreq);

  // config detail: {'description': 'Horn request', 'enum': {0:
  // 'PC_HORNREQ_NOREQ', 1: 'PC_HORNREQ_REQ'}, 'precision': 1.0, 'len': 1,
  // 'name': 'PC_HornReq', 'is_signed_var': False, 'offset': 0.0,
  // 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola',
  // 'physical_unit': ''}
  void set_p_pc_hornreq(uint8_t* data, Pc_bcm_201::Pc_hornreqType pc_hornreq);

  // config detail: {'description': 'Hazard lamp request', 'enum': {0:
  // 'PC_HAZARDLAMPREQ_NOREQ', 1: 'PC_HAZARDLAMPREQ_REQ'}, 'precision': 1.0,
  // 'len': 1, 'name': 'PC_HazardLampReq', 'is_signed_var': False, 'offset':
  // 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order':
  // 'motorola', 'physical_unit': ''}
  void set_p_pc_hazardlampreq(
      uint8_t* data, Pc_bcm_201::Pc_hazardlampreqType pc_hazardlampreq);

 private:
  Pc_bcm_201::Pc_reverselampreqType pc_reverselampreq_;
  Pc_bcm_201::Pc_lowbeamreqType pc_lowbeamreq_;
  Pc_bcm_201::Pc_highbeamreqType pc_highbeamreq_;
  Pc_bcm_201::Pc_rightturnlampreqType pc_rightturnlampreq_;
  Pc_bcm_201::Pc_leftturnlampreqType pc_leftturnlampreq_;
  Pc_bcm_201::Pc_hornreqType pc_hornreq_;
  Pc_bcm_201::Pc_hazardlampreqType pc_hazardlampreq_;
};

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
