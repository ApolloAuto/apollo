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

#include "modules/drivers/radar/racobit_radar/protocol/cluster_quality_info_702.h"
#include "modules/drivers/radar/racobit_radar/protocol/const_vars.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::canbus::Byte;

ClusterQualityInfo702::ClusterQualityInfo702() {}
const uint32_t ClusterQualityInfo702::ID = 0x702;

void ClusterQualityInfo702::Parse(const std::uint8_t* bytes, int32_t length,
                                  RacobitRadar* racobit_radar) const {
  int id = target_id(bytes, length);
  for (int i = 0; i < racobit_radar->contiobs_size(); ++i) {
    if (racobit_radar->contiobs(i).obstacle_id() == id) {
      auto racobit_obs = racobit_radar->mutable_contiobs(i);
      racobit_obs->set_longitude_dist_rms(
          LINEAR_RMS[longitude_dist_rms(bytes, length)]);
      racobit_obs->set_lateral_dist_rms(
          LINEAR_RMS[lateral_dist_rms(bytes, length)]);
      racobit_obs->set_longitude_vel_rms(
          LINEAR_RMS[longitude_vel_rms(bytes, length)]);
      racobit_obs->set_lateral_vel_rms(
          LINEAR_RMS[lateral_vel_rms(bytes, length)]);
      racobit_obs->set_probexist(PROBOFEXIST[pdh0(bytes, length)]);
      switch (invalid_state(bytes, length)) {
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x06:
        case 0x07:
        case 0x0E:
          racobit_obs->set_probexist(PROBOFEXIST[0]);
        default:
          break;
      }
      switch (ambig_state(bytes, length)) {
        case 0x00:
        case 0x01:
        case 0x02:
          racobit_obs->set_probexist(PROBOFEXIST[0]);
        default:
          break;
      }
    }
  }
}

int ClusterQualityInfo702::target_id(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::longitude_dist_rms(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::lateral_dist_rms(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::longitude_vel_rms(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 5);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::pdh0(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::ambig_state(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::invalid_state(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::lateral_vel_rms(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
