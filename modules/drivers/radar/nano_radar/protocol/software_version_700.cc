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

#include "modules/drivers/radar/nano_radar/protocol/software_version_700.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::canbus::Byte;

SoftwareVersion700::SoftwareVersion700() {}
const uint32_t SoftwareVersion700::ID = 0x700;

void SoftwareVersion700::Parse(const std::uint8_t* bytes, int32_t length,
                               NanoRadar* nano_radar) const {
  auto version = nano_radar->mutable_software_version();
  version->set_soft_major_release(soft_major_release(bytes, length));
  version->set_soft_minor_release(soft_minor_release(bytes, length));
  version->set_soft_patch_level(soft_patch_level(bytes, length));
}

int SoftwareVersion700::soft_major_release(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int SoftwareVersion700::soft_minor_release(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int SoftwareVersion700::soft_patch_level(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
