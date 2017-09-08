
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

#include "modules/drivers/delphi_esr/protocol/test_4e0.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using ::apollo::canbus::Byte;

const int Test4e0::ID = 0x4E0;

void Test4e0::Parse(const uint8_t *bytes, int32_t length,
                       DelphiESR *delphi_esr) const {
  delphi_esr->mutable_test_4e0()->set_scan_index(
      scan_index(bytes, length));
}

// config detail: {'name': 'num_obstacles', 'offset': 0.0, 'precision': 1.0,
// 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
int Test4e0::scan_index(const uint8_t *bytes, int32_t length) const {
  Byte t0(bytes + 3);
  Byte t1(bytes + 4);
  int32_t x = t0.get_byte(0, 8);
  int32_t y = t1.get_byte(0, 8) + (x << 8);

  return y;
}

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
