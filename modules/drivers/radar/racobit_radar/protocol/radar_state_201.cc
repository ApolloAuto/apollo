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

#include "modules/drivers/radar/racobit_radar/protocol/radar_state_201.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::canbus::Byte;

RadarState201::RadarState201() {}
const uint32_t RadarState201::ID = 0x201;

void RadarState201::Parse(const std::uint8_t* bytes, int32_t length,
                          RacobitRadar* racobit_radar) const {
  auto state = racobit_radar->mutable_radar_state();
  state->set_max_distance(max_dist(bytes, length));
  state->set_output_type(output_type(bytes, length));
  state->set_rcs_threshold(rcs_threshold(bytes, length));
  state->set_radar_power(radar_power(bytes, length));
  state->set_send_quality(send_quality(bytes, length));
  state->set_send_ext_info(send_ext_info(bytes, length));
}

int RadarState201::max_dist(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  uint32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  int ret = x * 2;
  return ret;
}

int RadarState201::radar_power(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 3);
  uint32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 4);
  uint32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

OutputType RadarState201::output_type(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(2, 2);

  switch (x) {
    case 0x0:
      return OUTPUT_TYPE_NONE;
    case 0x1:
      return OUTPUT_TYPE_OBJECTS;
    case 0x2:
      return OUTPUT_TYPE_CLUSTERS;
    default:
      return OUTPUT_TYPE_ERROR;
  }
}

RcsThreshold RadarState201::rcs_threshold(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(2, 3);

  switch (x) {
    case 0x0:
      return RCS_THRESHOLD_STANDARD;
    case 0x1:
      return RCS_THRESHOLD_HIGH_SENSITIVITY;
    default:
      return RCS_THRESHOLD_ERROR;
  }
}

bool RadarState201::send_quality(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(4, 1);

  bool ret = (x == 0x1);
  return ret;
}

bool RadarState201::send_ext_info(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(5, 1);

  bool ret = (x == 0x1);
  return ret;
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
