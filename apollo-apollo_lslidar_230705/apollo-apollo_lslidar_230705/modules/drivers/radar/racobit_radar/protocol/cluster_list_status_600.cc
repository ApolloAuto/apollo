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

#include "modules/drivers/radar/racobit_radar/protocol/cluster_list_status_600.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::RacobitRadarObs;
using apollo::drivers::canbus::Byte;

ClusterListStatus600::ClusterListStatus600() {}
const uint32_t ClusterListStatus600::ID = 0x600;

void ClusterListStatus600::Parse(const std::uint8_t* bytes, int32_t length,
                                 RacobitRadar* racobit_radar) const {
  auto status = racobit_radar->mutable_cluster_list_status();
  status->set_near(near(bytes, length));
  status->set_far(far(bytes, length));
  status->set_meas_counter(meas_counter(bytes, length));
  status->set_interface_version(interface_version(bytes, length));
  auto counter = status->near() + status->far();
  racobit_radar->mutable_contiobs()->Reserve(counter);
}

int ClusterListStatus600::near(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterListStatus600::far(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterListStatus600::meas_counter(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  uint32_t t = t0.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

int ClusterListStatus600::interface_version(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
