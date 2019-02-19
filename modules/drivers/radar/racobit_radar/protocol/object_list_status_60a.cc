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

#include "modules/drivers/radar/racobit_radar/protocol/object_list_status_60a.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::RacobitRadarObs;
using apollo::drivers::canbus::Byte;

ObjectListStatus60A::ObjectListStatus60A() {}
const uint32_t ObjectListStatus60A::ID = 0x60A;

void ObjectListStatus60A::Parse(const std::uint8_t* bytes, int32_t length,
                                RacobitRadar* racobit_radar) const {
  auto status = racobit_radar->mutable_object_list_status();
  auto num_of_obj = num_of_objects(bytes, length);
  status->set_nof_objects(num_of_obj);
  status->set_meas_counter(meas_counter(bytes, length));
  status->set_interface_version(interface_version(bytes, length));
  racobit_radar->mutable_contiobs()->Reserve(num_of_obj);
}

int ObjectListStatus60A::num_of_objects(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ObjectListStatus60A::meas_counter(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectListStatus60A::interface_version(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
