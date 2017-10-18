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

#include "modules/canbus/vehicle/lincoln/protocol/misc_69.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Misc69::ID = 0x69;

void Misc69::Parse(const std::uint8_t *bytes, int32_t length,
                   ChassisDetail *chassis_detail) const {
  int32_t turn_light_type = turn_signal_status(bytes, length);
  switch (turn_light_type) {
    case 0:
      chassis_detail->mutable_light()->set_turn_light_type(
          Light::TURN_LIGHT_OFF);
      break;
    case 1:
      chassis_detail->mutable_light()->set_turn_light_type(Light::TURN_LEFT_ON);
      break;
    case 2:
      chassis_detail->mutable_light()->set_turn_light_type(
          Light::TURN_RIGHT_ON);
      break;
    case 3:
      break;
    default:
      break;
  }

  int32_t hi_beam_status = high_beam_status(bytes, length);
  switch (hi_beam_status) {
    case 0:
      chassis_detail->mutable_light()->set_lincoln_lamp_type(Light::BEAM_NULL);
      break;
    case 1:
      chassis_detail->mutable_light()->set_lincoln_lamp_type(
          Light::BEAM_FLASH_TO_PASS);
      break;
    case 2:
      chassis_detail->mutable_light()->set_lincoln_lamp_type(Light::BEAM_HIGH);
      break;
    case 3:
    default:
      chassis_detail->mutable_light()->set_lincoln_lamp_type(
          Light::BEAM_INVALID);
      break;
  }

  // wiper status, non-compatible
  int32_t wiper = wiper_status(bytes, length);
  switch (wiper) {
    case 0:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_OFF);
      break;
    case 1:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_AUTO_OFF);
      break;
    case 2:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_OFF_MOVING);
      break;
    case 3:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_MANUAL_OFF);
      break;
    case 4:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_MANUAL_ON);
      break;
    case 5:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_MANUAL_LOW);
      break;
    case 6:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_MANUAL_HIGH);
      break;
    case 7:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_MIST_FLICK);
      break;
    case 8:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_WASH);
      break;
    case 9:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_AUTO_LOW);
      break;
    case 10:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_AUTO_HIGH);
      break;
    case 11:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_COURTESY_WIPE);
      break;
    case 12:
      chassis_detail->mutable_light()->set_lincoln_wiper(
          Light::WIPER_AUTO_ADJUST);
      break;
    case 13:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_RESERVED);
      break;
    case 14:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_STALLED);
      break;
    case 15:
      chassis_detail->mutable_light()->set_lincoln_wiper(Light::WIPER_NO_DATA);
      break;
  }

  int32_t ambient = ambient_light_status(bytes, length);
  switch (ambient) {
    case 0:
      chassis_detail->mutable_light()->set_lincoln_ambient(Light::AMBIENT_DARK);
      break;
    case 1:
      chassis_detail->mutable_light()->set_lincoln_ambient(
          Light::AMBIENT_LIGHT);
      break;
    case 2:
      chassis_detail->mutable_light()->set_lincoln_ambient(
          Light::AMBIENT_TWILIGHT);
      break;
    case 3:
      chassis_detail->mutable_light()->set_lincoln_ambient(
          Light::AMBIENT_TUNNEL_ON);
      break;
    case 4:
      chassis_detail->mutable_light()->set_lincoln_ambient(
          Light::AMBIENT_TUNNEL_OFF);
      break;
    case 7:
      chassis_detail->mutable_light()->set_lincoln_ambient(
          Light::AMBIENT_NO_DATA);
      break;
    default:
      chassis_detail->mutable_light()->set_lincoln_ambient(
          Light::AMBIENT_INVALID);
      break;
  }

  // acc button related
  chassis_detail->mutable_basic()->set_acc_on_button(
      is_acc_on_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_off_button(
      is_acc_off_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_res_button(
      is_acc_resume_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_cancel_button(
      is_acc_cancel_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_on_off_button(
      is_acc_on_or_off_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_res_cancel_button(
      is_acc_resume_or_cancel_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_inc_spd_button(
      is_acc_increment_set_speed_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_dec_spd_button(
      is_acc_decrement_set_speed_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_inc_gap_button(
      is_acc_increment_following_gap_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_acc_dec_gap_button(
      is_acc_decrement_following_gap_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_lka_button(
      is_lka_on_or_off_pressed(bytes, length));
  chassis_detail->mutable_basic()->set_canbus_fault(
      is_canbus_fault(bytes, length));

  // driver door
  if (is_driver_door_open(bytes, length)) {
    chassis_detail->mutable_safety()->set_is_driver_car_door_close(false);
  } else {
    chassis_detail->mutable_safety()->set_is_driver_car_door_close(true);
  }

  chassis_detail->mutable_safety()->set_is_passenger_door_open(
      is_passenger_door_open(bytes, length));
  chassis_detail->mutable_safety()->set_is_rearleft_door_open(
      is_rear_left_door_open(bytes, length));
  chassis_detail->mutable_safety()->set_is_rearright_door_open(
      is_rear_right_door_open(bytes, length));
  chassis_detail->mutable_safety()->set_is_hood_open(
      is_hood_open(bytes, length));
  chassis_detail->mutable_safety()->set_is_trunk_open(
      is_trunk_open(bytes, length));
  chassis_detail->mutable_safety()->set_is_passenger_detected(
      is_passenger_detected(bytes, length));
  chassis_detail->mutable_safety()->set_is_passenger_airbag_enabled(
      is_passenger_airbag_enabled(bytes, length));

  // airbag ? driver airbag?
  // chassis_detail->mutable_basic()->set_is_air_bag_deployed(
  //     is_passenger_airbag_enabled(bytes, length));

  // driver buckled
  chassis_detail->mutable_safety()->set_is_driver_buckled(
      is_driver_belt_buckled(bytes, length));
  chassis_detail->mutable_safety()->set_is_passenger_buckled(
      is_passenger_belt_buckled(bytes, length));

  // has error?, non-compatible
  // chassis_detail->mutable_safety()->set_has_error(
  //    is_canbus_fault(bytes, length));
}

int32_t Misc69::turn_signal_status(const std::uint8_t *bytes,
                                   int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(0, 2);
  return x;
}

int32_t Misc69::high_beam_status(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(2, 2);
  return x;
}

int32_t Misc69::wiper_status(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(4, 4);
  return x;
}

int32_t Misc69::ambient_light_status(const std::uint8_t *bytes,
                                     int32_t length) const {
  Byte frame(bytes + 1);
  int32_t x = frame.get_byte(0, 3);
  return x;
}

bool Misc69::is_acc_on_pressed(const std::uint8_t *bytes,
                               int32_t length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(3);
}

bool Misc69::is_acc_off_pressed(const std::uint8_t *bytes,
                                int32_t length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(4);
}

bool Misc69::is_acc_resume_pressed(const std::uint8_t *bytes,
                                   int32_t length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(5);
}

bool Misc69::is_acc_cancel_pressed(const std::uint8_t *bytes,
                                   int32_t length) const {
  Byte frame(bytes + 1);
  return frame.is_bit_1(6);
}

bool Misc69::is_acc_on_or_off_pressed(const std::uint8_t *bytes,
                                      int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(0);
}

bool Misc69::is_acc_resume_or_cancel_pressed(const std::uint8_t *bytes,
                                             int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(1);
}

bool Misc69::is_acc_increment_set_speed_pressed(const std::uint8_t *bytes,
                                                int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(2);
}

bool Misc69::is_acc_decrement_set_speed_pressed(const std::uint8_t *bytes,
                                                int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(3);
}

bool Misc69::is_acc_increment_following_gap_pressed(const std::uint8_t *bytes,
                                                    int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(4);
}

bool Misc69::is_acc_decrement_following_gap_pressed(const std::uint8_t *bytes,
                                                    int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(5);
}

bool Misc69::is_lka_on_or_off_pressed(const std::uint8_t *bytes,
                                      int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(6);
}

bool Misc69::is_canbus_fault(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 2);
  return frame.is_bit_1(7);
}

bool Misc69::is_driver_door_open(const std::uint8_t *bytes,
                                 int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(0);
}

bool Misc69::is_passenger_door_open(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(1);
}

bool Misc69::is_rear_left_door_open(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(2);
}

bool Misc69::is_rear_right_door_open(const std::uint8_t *bytes,
                                     int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(3);
}

bool Misc69::is_hood_open(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(4);
}

bool Misc69::is_trunk_open(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(5);
}

bool Misc69::is_passenger_detected(const std::uint8_t *bytes,
                                   int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(6);
}

bool Misc69::is_passenger_airbag_enabled(const std::uint8_t *bytes,
                                         int32_t length) const {
  Byte frame(bytes + 3);
  return frame.is_bit_1(7);
}

bool Misc69::is_driver_belt_buckled(const std::uint8_t *bytes,
                                    int32_t length) const {
  Byte frame(bytes + 4);
  return frame.is_bit_1(0);
}

bool Misc69::is_passenger_belt_buckled(const std::uint8_t *bytes,
                                       int32_t length) const {
  Byte frame(bytes + 4);
  return frame.is_bit_1(1);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
