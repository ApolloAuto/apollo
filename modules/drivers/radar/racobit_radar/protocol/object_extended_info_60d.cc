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

#include "modules/drivers/radar/racobit_radar/protocol/object_extended_info_60d.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/racobit_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::canbus::Byte;

ObjectExtendedInfo60D::ObjectExtendedInfo60D() {}
const uint32_t ObjectExtendedInfo60D::ID = 0x60D;

void ObjectExtendedInfo60D::Parse(const std::uint8_t* bytes, int32_t length,
                                  RacobitRadar* racobit_radar) const {
  int obj_id = object_id(bytes, length);
  for (int i = 0; i < racobit_radar->contiobs_size(); ++i) {
    if (racobit_radar->contiobs(i).obstacle_id() == obj_id) {
      auto obs = racobit_radar->mutable_contiobs(i);
      obs->set_longitude_accel(longitude_accel(bytes, length));
      obs->set_lateral_accel(lateral_accel(bytes, length));
      obs->set_oritation_angle(oritation_angle(bytes, length));
      obs->set_length(object_length(bytes, length));
      obs->set_width(object_width(bytes, length));
      obs->set_obstacle_class(obstacle_class(bytes, length));
      break;
    }
  }
  // auto conti_obs = racobit_radar->mutable_contiobs(object_id(bytes, length));
}

int ObjectExtendedInfo60D::object_id(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

double ObjectExtendedInfo60D::longitude_accel(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  double ret = x * OBJECT_AREL_RES + OBJECT_AREL_LONG_MIN;
  return ret;
}

double ObjectExtendedInfo60D::lateral_accel(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  double ret = x * OBJECT_AREL_RES + OBJECT_AREL_LAT_MIN;
  return ret;
}

int ObjectExtendedInfo60D::obstacle_class(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

double ObjectExtendedInfo60D::oritation_angle(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;

  double ret = x * OBJECT_ORIENTATION_ANGEL_RES + OBJECT_ORIENTATION_ANGEL_MIN;
  return ret;
}

double ObjectExtendedInfo60D::object_length(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_LENGTH_RES;
  return ret;
}

double ObjectExtendedInfo60D::object_width(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_WIDTH_RES;
  return ret;
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
