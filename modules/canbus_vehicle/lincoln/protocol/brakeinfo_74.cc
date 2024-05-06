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

#include "modules/canbus_vehicle/lincoln/protocol/brakeinfo_74.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakeinfo74::ID = 0x74;

void Brakeinfo74::Parse(const std::uint8_t *bytes, int32_t length,
                        Lincoln *chassis_detail) const {
  chassis_detail->mutable_brake()->set_brake_torque_req(
      braking_torque_request(bytes, length));
  switch (hill_start_assist_status(bytes, length)) {
    case 0:
      chassis_detail->mutable_brake()->set_hsa_status(Brake::HSA_INACTIVE);
      break;
    case 1:
      chassis_detail->mutable_brake()->set_hsa_status(
          Brake::HSA_FINDING_GRADIENT);
      break;
    case 2:
      chassis_detail->mutable_brake()->set_hsa_status(
          Brake::HSA_ACTIVE_PRESSED);
      break;
    case 3:
      chassis_detail->mutable_brake()->set_hsa_status(
          Brake::HSA_ACTIVE_RELEASED);
      break;
    case 4:
      chassis_detail->mutable_brake()->set_hsa_status(Brake::HSA_FAST_RELEASE);
      break;
    case 5:
      chassis_detail->mutable_brake()->set_hsa_status(Brake::HSA_SLOW_RELEASE);
      break;
    case 6:
      chassis_detail->mutable_brake()->set_hsa_status(Brake::HSA_FAILED);
      break;
    case 7:
      chassis_detail->mutable_brake()->set_hsa_status(Brake::HSA_UNDEFINED);
      break;
  }
  chassis_detail->mutable_vehicle_spd()->set_is_vehicle_standstill(
      is_vehicle_stationary(bytes, length));
  chassis_detail->mutable_brake()->set_brake_torque_act(
      braking_torque_actual(bytes, length));
  switch (hill_start_assist_mode(bytes, length)) {
    case 0:
      chassis_detail->mutable_brake()->set_hsa_mode(Brake::HSA_OFF);
      break;
    case 1:
      chassis_detail->mutable_brake()->set_hsa_mode(Brake::HSA_AUTO);
      break;
    case 2:
      chassis_detail->mutable_brake()->set_hsa_mode(Brake::HSA_MANUAL);
      break;
    case 3:
      chassis_detail->mutable_brake()->set_hsa_mode(Brake::HSA_MODE_UNDEFINED);
      break;
  }
  switch (parking_brake_status(bytes, length)) {
    case 0:
      chassis_detail->mutable_epb()->set_parking_brake_status(Epb::PBRAKE_OFF);
      break;
    case 1:
      chassis_detail->mutable_epb()->set_parking_brake_status(
          Epb::PBRAKE_TRANSITION);
      break;
    case 2:
      chassis_detail->mutable_epb()->set_parking_brake_status(Epb::PBRAKE_ON);
      break;
    case 3:
      chassis_detail->mutable_epb()->set_parking_brake_status(
          Epb::PBRAKE_FAULT);
      break;
  }
  chassis_detail->mutable_brake()->set_wheel_torque_act(
      wheel_torque_actual(bytes, length));
  chassis_detail->mutable_vehicle_spd()->set_acc_est(
      acceleration_over_ground(bytes, length));
  chassis_detail->mutable_esp()->set_is_abs_active(
      is_abs_active(bytes, length));
  chassis_detail->mutable_esp()->set_is_abs_enabled(
      is_abs_enabled(bytes, length));
  chassis_detail->mutable_esp()->set_is_stab_active(
      is_stability_control_active(bytes, length));
  chassis_detail->mutable_esp()->set_is_stab_enabled(
      is_stability_control_enabled(bytes, length));
  chassis_detail->mutable_esp()->set_is_trac_active(
      is_traction_control_active(bytes, length));
  chassis_detail->mutable_esp()->set_is_trac_enabled(
      is_traction_control_enabled(bytes, length));
}

double Brakeinfo74::braking_torque_request(const std::uint8_t *bytes,
                                           int32_t length) const {
  Byte frame_high(bytes + 1);
  int32_t high = frame_high.get_byte(0, 4);
  Byte frame_low(bytes + 0);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 4.000000;
}

int32_t Brakeinfo74::hill_start_assist_status(const std::uint8_t *bytes,
                                              int32_t length) const {
  // see table for status code
  Byte frame(bytes + 1);
  int32_t x = frame.get_byte(4, 3);
  return x;
}

bool Brakeinfo74::is_vehicle_stationary(const std::uint8_t *bytes,
                                        int32_t length) const {
  // false for moving, true for stationary
  Byte frame(bytes + 1);
  return frame.is_bit_1(7);
}

double Brakeinfo74::braking_torque_actual(const std::uint8_t *bytes,
                                          int32_t length) const {
  Byte frame_high(bytes + 3);
  int32_t high = frame_high.get_byte(0, 4);
  Byte frame_low(bytes + 2);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 4.000000;
}

int32_t Brakeinfo74::hill_start_assist_mode(const std::uint8_t *bytes,
                                            int32_t length) const {
  // see table for status code
  Byte frame(bytes + 3);
  int32_t x = frame.get_byte(4, 2);
  return x;
}

int32_t Brakeinfo74::parking_brake_status(const std::uint8_t *bytes,
                                          int32_t length) const {
  // see table for status code
  Byte frame(bytes + 3);
  int32_t x = frame.get_byte(6, 2);
  return x;
}

double Brakeinfo74::wheel_torque_actual(const std::uint8_t *bytes,
                                        int32_t length) const {
  Byte frame_high(bytes + 5);
  int32_t high = frame_high.get_byte(0, 6);
  Byte frame_low(bytes + 4);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x1FFF) {
    value -= 0x4000;
  }
  return value * 4.000000;
}

double Brakeinfo74::acceleration_over_ground(const std::uint8_t *bytes,
                                             int32_t length) const {
  // vehicle acceleration over ground estimate
  Byte frame_high(bytes + 7);
  int32_t high = frame_high.get_byte(0, 2);
  Byte frame_low(bytes + 6);
  int32_t low = frame_low.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  if (value > 0x1FF) {
    value -= 0x400;
  }
  return value * 0.035000;
}

bool Brakeinfo74::is_abs_active(const std::uint8_t *bytes,
                                int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(2);
}

bool Brakeinfo74::is_abs_enabled(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

bool Brakeinfo74::is_stability_control_active(const std::uint8_t *bytes,
                                              int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(4);
}

bool Brakeinfo74::is_stability_control_enabled(const std::uint8_t *bytes,
                                               int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(5);
}

bool Brakeinfo74::is_traction_control_active(const std::uint8_t *bytes,
                                             int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

bool Brakeinfo74::is_traction_control_enabled(const std::uint8_t *bytes,
                                              int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
