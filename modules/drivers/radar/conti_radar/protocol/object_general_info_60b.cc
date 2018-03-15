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

#include "modules/drivers/radar/conti_radar/protocol/object_general_info_60b.h"

#include "glog/logging.h"

#include "modules/common/time/time.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/conti_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace conti_radar {

using apollo::drivers::canbus::Byte;

ObjectGeneralInfo60B::ObjectGeneralInfo60B() {}
const uint32_t ObjectGeneralInfo60B::ID = 0x60B;

void ObjectGeneralInfo60B::Parse(const std::uint8_t* bytes, int32_t length,
                                 ContiRadar* conti_radar) const {
  int obj_id = object_id(bytes, length);
  auto conti_obs = conti_radar->add_contiobs();
  conti_obs->set_clusterortrack(false);
  conti_obs->set_obstacle_id(obj_id);
  conti_obs->set_longitude_dist(longitude_dist(bytes, length));
  conti_obs->set_lateral_dist(lateral_dist(bytes, length));
  conti_obs->set_longitude_vel(longitude_vel(bytes, length));
  conti_obs->set_lateral_vel(lateral_vel(bytes, length));
  conti_obs->set_rcs(rcs(bytes, length));
  conti_obs->set_dynprop(dynprop(bytes, length));
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  auto header = conti_obs->mutable_header();
  header->CopyFrom(conti_radar->header());
  header->set_timestamp_sec(timestamp);
}

int ObjectGeneralInfo60B::object_id(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

double ObjectGeneralInfo60B::longitude_dist(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);

  x <<= 5;
  x |= t;

  double ret = x * OBJECT_DIST_RES + OBJECT_DIST_LONG_MIN;
  return ret;
}

double ObjectGeneralInfo60B::lateral_dist(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x * OBJECT_DIST_RES + OBJECT_DIST_LAT_MIN;
  return ret;
}

double ObjectGeneralInfo60B::longitude_vel(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);
  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;
  double ret = x * OBJECT_VREL_RES + OBJECT_VREL_LONG_MIN;
  return ret;
}

double ObjectGeneralInfo60B::lateral_vel(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  double ret = x * OBJECT_VREL_RES + OBJECT_VREL_LAT_MIN;
  return ret;
}

double ObjectGeneralInfo60B::rcs(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_RCS_RES + OBJECT_RCS_MIN;
  return ret;
}

int ObjectGeneralInfo60B::dynprop(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo
