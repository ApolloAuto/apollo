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

#include "modules/drivers/delphi_esr/protocol/esr_trackmotionpower_540.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace delphi_esr {

using apollo::drivers::canbus::Byte;

Esrtrackmotionpower540::Esrtrackmotionpower540() {}
const int32_t Esrtrackmotionpower540::ID = 0x540;

void Esrtrackmotionpower540::Parse(const std::uint8_t* bytes, int32_t length,
                                   DelphiESR* delphi_esr) const {
  auto* esr_trackmotionpower_540 = delphi_esr->add_esr_trackmotionpower_540();
  esr_trackmotionpower_540->set_can_tx_track_rolling_count_2(
      can_tx_track_rolling_count_2(bytes, length));
  esr_trackmotionpower_540->set_can_tx_track_can_id_group(
      can_tx_track_can_id_group(bytes, length));
  for (int32_t index = 0;
       index <
       (esr_trackmotionpower_540->can_tx_track_can_id_group() < 9 ? 7 : 1);
       ++index) {
    auto* can_tx_track_motion_power =
        esr_trackmotionpower_540->add_can_tx_track_motion_power();
    can_tx_track_motion_power->set_can_tx_track_moving(
        can_tx_track_moving(bytes, length, index));
    can_tx_track_motion_power->set_can_tx_track_moving_fast(
        can_tx_track_moving_fast(bytes, length, index));
    can_tx_track_motion_power->set_can_tx_track_moving_slow(
        can_tx_track_moving_slow(bytes, length, index));
    can_tx_track_motion_power->set_can_tx_track_power(
        can_tx_track_power(bytes, length, index));
  }
}

// config detail: {'name': 'can_tx_track_rolling_count_2', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 4, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Esrtrackmotionpower540::can_tx_track_rolling_count_2(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

int32_t Esrtrackmotionpower540::can_tx_track_can_id_group(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  int32_t ret = x;
  return ret;
}

bool Esrtrackmotionpower540::can_tx_track_moving(const std::uint8_t* bytes,
                                                 int32_t length,
                                                 int32_t index) const {
  Byte t0(bytes + 1 + index);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

bool Esrtrackmotionpower540::can_tx_track_moving_fast(const std::uint8_t* bytes,
                                                      int32_t length,
                                                      int32_t index) const {
  Byte t0(bytes + 1 + index);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

bool Esrtrackmotionpower540::can_tx_track_moving_slow(const std::uint8_t* bytes,
                                                      int32_t length,
                                                      int32_t index) const {
  Byte t0(bytes + 1 + index);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

int32_t Esrtrackmotionpower540::can_tx_track_power(const std::uint8_t* bytes,
                                                   int32_t length,
                                                   int32_t index) const {
  Byte t0(bytes + 1 + index);
  int32_t x = t0.get_byte(0, 5);

  int32_t ret = x;
  return ret;
}

}  // namespace delphi_esr
}  // namespace drivers
}  // namespace apollo
