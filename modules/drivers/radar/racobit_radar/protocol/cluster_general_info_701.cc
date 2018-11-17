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

#include "modules/drivers/radar/racobit_radar/protocol/cluster_general_info_701.h"

#include "glog/logging.h"

#include "modules/common/time/time.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/racobit_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::canbus::Byte;

ClusterGeneralInfo701::ClusterGeneralInfo701() {}
const uint32_t ClusterGeneralInfo701::ID = 0x701;

void ClusterGeneralInfo701::Parse(const std::uint8_t* bytes, int32_t length,
                                  RacobitRadar* racobit_radar) const {
  auto obs = racobit_radar->add_contiobs();
  obs->set_clusterortrack(true);
  obs->set_obstacle_id(obstacle_id(bytes, length));
  obs->set_longitude_dist(longitude_dist(bytes, length));
  obs->set_lateral_dist(lateral_dist(bytes, length));
  obs->set_longitude_vel(longitude_vel(bytes, length));
  obs->set_lateral_vel(lateral_vel(bytes, length));
  obs->set_rcs(rcs(bytes, length));
  obs->set_dynprop(dynprop(bytes, length));
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  auto header = obs->mutable_header();
  header->CopyFrom(racobit_radar->header());
  header->set_timestamp_sec(timestamp);
}

int ClusterGeneralInfo701::obstacle_id(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

double ClusterGeneralInfo701::longitude_dist(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  uint32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;
  double ret = x * CLUSTER_DIST_RES + CLUSTER_DIST_LONG_MIN;
  return ret;
}

double ClusterGeneralInfo701::lateral_dist(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 3);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  double ret = x * CLUSTER_DIST_RES + CLUSTER_DIST_LAT_MIN;
  return ret;
}

double ClusterGeneralInfo701::longitude_vel(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  uint32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;
  double ret = x * CLUSTER_VREL_RES + CLUSTER_VREL_LONG_MIN;
  return ret;
}

double ClusterGeneralInfo701::lateral_vel(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 6);
  uint32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;
  double ret = x * CLUSTER_VREL_RES + CLUSTER_VREL_LAT_MIN;
  return ret;
}

double ClusterGeneralInfo701::rcs(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(0, 8);
  double ret = x * CLUSTER_RCS_RES + CLUSTER_RCS;
  return ret;
}

int ClusterGeneralInfo701::dynprop(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 6);
  uint32_t x = t0.get_byte(0, 3);
  int ret = x;
  return ret;
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
