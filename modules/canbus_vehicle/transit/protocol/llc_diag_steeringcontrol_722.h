/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/transit/proto/transit.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transit {

class Llcdiagsteeringcontrol722
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Transit> {
 public:
  static const int32_t ID;

  Llcdiagsteeringcontrol722();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 0.0002, 'len': 16, 'name':
  // 'LLC_DBG_SteeringSensorPosition', 'is_signed_var': True, 'physical_range':
  // '[-6.5536|6.5534]', 'bit': 40, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'rev'}
  Llcdiagsteeringcontrol722* set_llc_dbg_steeringsensorposition(
      double llc_dbg_steeringsensorposition);

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name':
  // 'LLC_DBG_SteeringRackInputTorque', 'is_signed_var': True, 'physical_range':
  // '[-32768|32767]', 'bit': 24, 'type': 'int', 'order': 'intel',
  // 'physical_unit': 'counts'}
  Llcdiagsteeringcontrol722* set_llc_dbg_steeringrackinputtorque(
      int llc_dbg_steeringrackinputtorque);

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 1e-05, 'len': 24, 'name':
  // 'LLC_DBG_SteeringMotorPosition', 'is_signed_var': True, 'physical_range':
  // '[-83.88608|83.88607]', 'bit': 0, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'rev'}
  Llcdiagsteeringcontrol722* set_llc_dbg_steeringmotorposition(
      double llc_dbg_steeringmotorposition);

 private:
  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 0.0002, 'len': 16, 'name':
  // 'LLC_DBG_SteeringSensorPosition', 'is_signed_var': True, 'physical_range':
  // '[-6.5536|6.5534]', 'bit': 40, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'rev'}
  void set_p_llc_dbg_steeringsensorposition(
      uint8_t* data, double llc_dbg_steeringsensorposition);

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 1.0, 'len': 16, 'name':
  // 'LLC_DBG_SteeringRackInputTorque', 'is_signed_var': True, 'physical_range':
  // '[-32768|32767]', 'bit': 24, 'type': 'int', 'order': 'intel',
  // 'physical_unit': 'counts'}
  void set_p_llc_dbg_steeringrackinputtorque(
      uint8_t* data, int llc_dbg_steeringrackinputtorque);

  // config detail: {'description': 'Brake control feedforward contribution',
  // 'offset': 0.0, 'precision': 1e-05, 'len': 24, 'name':
  // 'LLC_DBG_SteeringMotorPosition', 'is_signed_var': True, 'physical_range':
  // '[-83.88608|83.88607]', 'bit': 0, 'type': 'double', 'order': 'intel',
  // 'physical_unit': 'rev'}
  void set_p_llc_dbg_steeringmotorposition(
      uint8_t* data, double llc_dbg_steeringmotorposition);

 private:
  double llc_dbg_steeringsensorposition_;
  int llc_dbg_steeringrackinputtorque_;
  double llc_dbg_steeringmotorposition_;
};

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
