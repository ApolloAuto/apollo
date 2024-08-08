/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/nano_radar/protocol/region_config_401.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/radar/nano_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::NanoRadar;
using apollo::drivers::canbus::Byte;

const uint32_t RegionConfig401::ID = 0x401;

RegionConfig401::RegionConfig401() {}
RegionConfig401::~RegionConfig401() {}

uint32_t RegionConfig401::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void RegionConfig401::UpdateData(uint8_t* data) {
  set_collision_detection_coordinates_valid_p(
      data, radar_conf_.collision_detection_coordinates_valid());
  set_collision_detection_activation_valid_p(
      data, radar_conf_.collision_detection_activation_valid());

  set_region_max_output_number_p(
      data, static_cast<uint8_t>(radar_conf_.region_max_output_number()));
  set_region_id_p(data, static_cast<uint8_t>(radar_conf_.region_id()));
  set_point1_longitude_p(data,
                         static_cast<double>(radar_conf_.point1_longitude()));
  set_point1_lateral_p(data, static_cast<double>(radar_conf_.point1_lateral()));
  set_point2_longitude_p(data,
                         static_cast<double>(radar_conf_.point2_longitude()));
  set_point2_lateral_p(data, static_cast<double>(radar_conf_.point2_lateral()));
}

/**
 * @brief reset the private variables
 */
void RegionConfig401::Reset() {
  radar_conf_.set_collision_detection_coordinates_valid(false);
  radar_conf_.set_collision_detection_activation_valid(false);

  radar_conf_.set_region_max_output_number(63);
  radar_conf_.set_region_id(1);
  radar_conf_.set_point1_longitude(0);
  radar_conf_.set_point1_lateral(50);
  radar_conf_.set_point2_longitude(20);
  radar_conf_.set_point2_lateral(-50);
}

RadarConf RegionConfig401::radar_conf() { return radar_conf_; }

RegionConfig401* RegionConfig401::set_radar_conf(RadarConf radar_conf) {
  radar_conf_.CopyFrom(radar_conf);
  return this;
}

RegionConfig401* RegionConfig401::set_collision_detection_coordinates_valid(
    bool valid) {
  radar_conf_.set_collision_detection_coordinates_valid(valid);
  return this;
}

RegionConfig401* RegionConfig401::set_collision_detection_activation_valid(
    bool valid) {
  radar_conf_.set_collision_detection_activation_valid(valid);
  return this;
}

RegionConfig401* RegionConfig401::set_region_max_output_number(uint8_t data) {
  radar_conf_.set_region_max_output_number(data);
  return this;
}

RegionConfig401* RegionConfig401::set_region_id(uint8_t data) {
  radar_conf_.set_region_id(data);
  return this;
}

RegionConfig401* RegionConfig401::set_point1_longitude(double data) {
  radar_conf_.set_point1_longitude(data);
  return this;
}

RegionConfig401* RegionConfig401::set_point1_lateral(double data) {
  radar_conf_.set_point1_lateral(data);
  return this;
}

RegionConfig401* RegionConfig401::set_point2_longitude(double data) {
  radar_conf_.set_point2_longitude(data);
  return this;
}

RegionConfig401* RegionConfig401::set_point2_lateral(double data) {
  radar_conf_.set_point2_lateral(data);
  return this;
}

void RegionConfig401::set_collision_detection_coordinates_valid_p(uint8_t* data,
                                                                  bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 7, 1);
  } else {
    frame.set_value(0, 7, 1);
  }
}

void RegionConfig401::set_collision_detection_activation_valid_p(uint8_t* data,
                                                                 bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 6, 1);
  } else {
    frame.set_value(0, 6, 1);
  }
}

void RegionConfig401::set_region_max_output_number_p(uint8_t* data,
                                                     uint8_t value) {
  Byte frame(data);
  frame.set_value(value, 0, 6);
}

void RegionConfig401::set_region_id_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 1);
  frame.set_value(value, 0, 3);
}

void RegionConfig401::set_point1_longitude_p(uint8_t* data, double value) {
  uint16_t x = (value - REGION_POINT_LONG_MIN) / REGION_POINT_RES;
  uint8_t t = 0;

  t = x & 0x1F;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 3, 5);

  x >>= 5;
  t = x & 0xFF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}

void RegionConfig401::set_point1_lateral_p(uint8_t* data, double value) {
  uint16_t x = (value - REGION_POINT_LAT_MIN) / REGION_POINT_RES;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);

  x >>= 8;
  t = x & 0x7;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 3);
}

void RegionConfig401::set_point2_longitude_p(uint8_t* data, double value) {
  uint16_t x = (value - REGION_POINT_LONG_MIN) / REGION_POINT_RES;
  uint8_t t = 0;

  t = x & 0x1F;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 3, 5);

  x >>= 5;
  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}

void RegionConfig401::set_point2_lateral_p(uint8_t* data, double value) {
  uint16_t x = ceil((value - REGION_POINT_LAT_MIN) / REGION_POINT_RES);
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 7);
  to_set0.set_value(t, 0, 8);

  x >>= 8;
  t = x & 0x7;
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 3);
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
