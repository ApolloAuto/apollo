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

#include "modules/canbus_vehicle/lincoln/protocol/surround_73.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::Byte;

const int32_t Surround73::ID = 0x73;

void Surround73::Parse(const std::uint8_t *bytes, int32_t length,
                       Lincoln *chassis_detail) const {
  // sonar left
  chassis_detail->mutable_surround()->set_cross_traffic_alert_left(
      is_cross_traffic_alert_left(bytes, length));
  chassis_detail->mutable_surround()->set_cross_traffic_alert_left_enabled(
      is_cross_traffic_alert_left_enabled(bytes, length));

  // blind spot left
  chassis_detail->mutable_surround()->set_blind_spot_left_alert(
      is_blind_spot_left_alert(bytes, length));
  chassis_detail->mutable_surround()->set_blind_spot_left_alert_enabled(
      is_blind_spot_left_alert_enabled(bytes, length));

  // sonar right
  chassis_detail->mutable_surround()->set_cross_traffic_alert_right(
      is_cross_traffic_alert_right(bytes, length));
  chassis_detail->mutable_surround()->set_cross_traffic_alert_left_enabled(
      is_cross_traffic_alert_left_enabled(bytes, length));

  // blind spot right
  chassis_detail->mutable_surround()->set_blind_spot_right_alert(
      is_blind_spot_right_alert(bytes, length));
  chassis_detail->mutable_surround()->set_blind_spot_right_alert_enabled(
      is_blind_spot_right_alert_enabled(bytes, length));

  // sonar00 ~ sonar11, output in meters
  chassis_detail->mutable_surround()->set_sonar00(sonar00(bytes, length));
  chassis_detail->mutable_surround()->set_sonar01(sonar01(bytes, length));
  chassis_detail->mutable_surround()->set_sonar02(sonar02(bytes, length));
  chassis_detail->mutable_surround()->set_sonar03(sonar03(bytes, length));
  chassis_detail->mutable_surround()->set_sonar04(sonar04(bytes, length));
  chassis_detail->mutable_surround()->set_sonar05(sonar05(bytes, length));
  chassis_detail->mutable_surround()->set_sonar06(sonar06(bytes, length));
  chassis_detail->mutable_surround()->set_sonar07(sonar07(bytes, length));
  chassis_detail->mutable_surround()->set_sonar08(sonar08(bytes, length));
  chassis_detail->mutable_surround()->set_sonar09(sonar09(bytes, length));
  chassis_detail->mutable_surround()->set_sonar10(sonar10(bytes, length));
  chassis_detail->mutable_surround()->set_sonar11(sonar11(bytes, length));

  // alternative representations.
  const int8_t kSonarNumbers = 12;
  chassis_detail->mutable_surround()->clear_sonar_range();
  for (std::int8_t i = 0; i < kSonarNumbers; ++i) {
    chassis_detail->mutable_surround()->add_sonar_range(
        sonars(bytes, i, length / 2));
  }
}

bool Surround73::is_cross_traffic_alert_left(const std::uint8_t *bytes,
                                             int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(0);
}

bool Surround73::is_cross_traffic_alert_left_enabled(const std::uint8_t *bytes,
                                                     int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(1);
}

bool Surround73::is_blind_spot_left_alert(const std::uint8_t *bytes,
                                          int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(2);
}

bool Surround73::is_blind_spot_left_alert_enabled(const std::uint8_t *bytes,
                                                  int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(3);
}

bool Surround73::is_cross_traffic_alert_right(const std::uint8_t *bytes,
                                              int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(4);
}

bool Surround73::is_cross_traffic_alert_right_enabled(const std::uint8_t *bytes,
                                                      int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(5);
}

bool Surround73::is_blind_spot_right_alert(const std::uint8_t *bytes,
                                           int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(6);
}

bool Surround73::is_blind_spot_right_alert_enabled(const std::uint8_t *bytes,
                                                   int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(7);
}

double Surround73::sonar00(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 1);
  int32_t x = frame.get_byte(0, 3);
  return sonar_range(x);
}

double Surround73::sonar01(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 1);
  int32_t x = frame.get_byte(4, 7);
  return sonar_range(x);
}

double Surround73::sonar02(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 2);
  int32_t x = frame.get_byte(0, 3);
  return sonar_range(x);
}

double Surround73::sonar03(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 2);
  int32_t x = frame.get_byte(4, 7);
  return sonar_range(x);
}

double Surround73::sonar04(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 3);
  int32_t x = frame.get_byte(0, 3);
  return sonar_range(x);
}

double Surround73::sonar05(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 3);
  int32_t x = frame.get_byte(4, 7);
  return sonar_range(x);
}

double Surround73::sonar06(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 4);
  int32_t x = frame.get_byte(0, 3);
  return sonar_range(x);
}

double Surround73::sonar07(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 4);
  int32_t x = frame.get_byte(4, 7);
  return sonar_range(x);
}

double Surround73::sonar08(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 5);
  int32_t x = frame.get_byte(0, 3);
  return sonar_range(x);
}

double Surround73::sonar09(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 5);
  int32_t x = frame.get_byte(4, 7);
  return sonar_range(x);
}

double Surround73::sonar10(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(0, 3);
  return sonar_range(x);
}

double Surround73::sonar11(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 6);
  int32_t x = frame.get_byte(4, 7);
  return sonar_range(x);
}

bool Surround73::sonar_enabled(const std::uint8_t *bytes,
                               int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(6);
}

bool Surround73::sonar_fault(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(7);
}

double Surround73::sonar_range(const std::int32_t x) const {
  if (x == 0x0) {
    return 100.0;  // If nothing detected, set test range to 100.0m.
  }
  return 0.145 * (x - 0x1) + 0.3;
}

double Surround73::sonars(const std::uint8_t *bytes, std::uint8_t sonar_number,
                          int32_t length) const {
  Byte frame(bytes + sonar_number / 2 + 1);
  int32_t start = (sonar_number % 2) * length;
  int32_t x = frame.get_byte(start, start + length);
  return sonar_range(x);
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
