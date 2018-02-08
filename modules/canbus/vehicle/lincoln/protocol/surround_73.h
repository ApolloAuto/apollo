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

/**
 * @file surround_73.h
 * @brief the class of Surround73 (for lincoln vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_LINCOLN_PROTOCOL_SURROUND_73_H_
#define MODULES_CANBUS_VEHICL_LINCOLN_PROTOCOL_SURROUND_73_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

/**
 * @class Surround73
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Surround73 : public ::apollo::drivers::canbus::ProtocolData<
                       ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param chassis_detail the parsed chassis_detail
   */
  virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     ChassisDetail *chassis_detail) const;

 private:
  bool is_cross_traffic_alert_left(const std::uint8_t *bytes,
                                   int32_t length) const;

  bool is_cross_traffic_alert_left_enabled(const std::uint8_t *bytes,
                                           int32_t length) const;

  bool is_blind_spot_left_alert(const std::uint8_t *bytes,
                                int32_t length) const;

  bool is_blind_spot_left_alert_enabled(const std::uint8_t *bytes,
                                        int32_t length) const;

  bool is_cross_traffic_alert_right(const std::uint8_t *bytes,
                                    int32_t length) const;

  bool is_cross_traffic_alert_right_enabled(const std::uint8_t *bytes,
                                            int32_t length) const;

  bool is_blind_spot_right_alert(const std::uint8_t *bytes,
                                 int32_t length) const;

  bool is_blind_spot_right_alert_enabled(const std::uint8_t *bytes,
                                         int32_t length) const;

  double sonar00(const std::uint8_t *bytes, int32_t length) const;

  double sonar01(const std::uint8_t *bytes, int32_t length) const;

  double sonar02(const std::uint8_t *bytes, int32_t length) const;

  double sonar03(const std::uint8_t *bytes, int32_t length) const;

  double sonar04(const std::uint8_t *bytes, int32_t length) const;

  double sonar05(const std::uint8_t *bytes, int32_t length) const;

  double sonar06(const std::uint8_t *bytes, int32_t length) const;

  double sonar07(const std::uint8_t *bytes, int32_t length) const;

  double sonar08(const std::uint8_t *bytes, int32_t length) const;

  double sonar09(const std::uint8_t *bytes, int32_t length) const;

  double sonar10(const std::uint8_t *bytes, int32_t length) const;

  double sonar11(const std::uint8_t *bytes, int32_t length) const;

  bool sonar_enabled(const std::uint8_t *bytes, int32_t length) const;

  bool sonar_fault(const std::uint8_t *bytes, int32_t length) const;

  double sonar_range(const std::int32_t x) const;
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_LINCOLN_PROTOCOL_SURROUND_73_H_
