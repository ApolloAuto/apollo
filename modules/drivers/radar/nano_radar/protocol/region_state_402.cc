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

// #include "math.h"
#include "modules/drivers/radar/nano_radar/protocol/region_state_402.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/nano_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::canbus::Byte;

CollisionDetectionRegionState402::CollisionDetectionRegionState402() {}
const uint32_t CollisionDetectionRegionState402::ID = 0x402;

void CollisionDetectionRegionState402::Parse(const std::uint8_t* bytes,
                                             int32_t length,
                                             NanoRadar* nano_radar) const {
  auto state = nano_radar->mutable_radar_region_state();
  state->set_region_id(region_id(bytes, length));
  state->set_region_max_output_number(region_max_output_number(bytes, length));
  state->set_point1_longitude(point1_longitude(bytes, length));
  state->set_point1_lateral(floor(point1_lateral(bytes, length)));
  state->set_point2_longitude(point2_longitude(bytes, length));
  state->set_point2_lateral(floor(point2_lateral(bytes, length)));
}

int CollisionDetectionRegionState402::region_id(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

int CollisionDetectionRegionState402::region_max_output_number(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(0, 5);

  int ret = x;
  return ret;
}

double CollisionDetectionRegionState402::point1_longitude(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);

  x <<= 5;
  x |= t;

  double ret = x * REGION_POINT_RES + REGION_POINT_LONG_MIN;
  return ret;
}

double CollisionDetectionRegionState402::point1_lateral(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x * REGION_POINT_RES + REGION_POINT_LAT_MIN;
  return ret;
}

double CollisionDetectionRegionState402::point2_longitude(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(3, 5);

  x <<= 5;
  x |= t;

  double ret = x * REGION_POINT_RES + REGION_POINT_LONG_MIN;
  return ret;
}

double CollisionDetectionRegionState402::point2_lateral(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x * REGION_POINT_RES + REGION_POINT_LAT_MIN;
  return ret;
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
