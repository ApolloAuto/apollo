/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/minibus/protocol/parkingmode_feedback_18ff8dac.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::drivers::canbus::Byte;

Parkingmodefeedback18ff8dac::Parkingmodefeedback18ff8dac() {}
const int32_t Parkingmodefeedback18ff8dac::ID = 0x38ff8dac;

void Parkingmodefeedback18ff8dac::Parse(const std::uint8_t* bytes,
                                        int32_t length,
                                        ChassisDetail* chassis) const {
  chassis->mutable_minibus()
      ->mutable_parkingmode_feedback_18ff8dac()
      ->set_pmf_current_status(pmf_current_status(bytes, length));
  chassis->mutable_minibus()
      ->mutable_parkingmode_feedback_18ff8dac()
      ->set_pmf_epb_onoff(pmf_epb_onoff(bytes, length));
}

// config detail: {'bit': 4, 'enum': {0: 'PMF_CURRENT_STATUS_EPB_RELEASE', 1:
// 'PMF_CURRENT_STATUS_EPB_TRIGGER', 2: 'PMF_CURRENT_STATUS_EPB_ACTION'},
// 'is_signed_var': False, 'len': 4, 'name': 'pmf_current_status', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Parkingmode_feedback_18ff8dac::Pmf_current_statusType
Parkingmodefeedback18ff8dac::pmf_current_status(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 4);

  Parkingmode_feedback_18ff8dac::Pmf_current_statusType ret =
      static_cast<Parkingmode_feedback_18ff8dac::Pmf_current_statusType>(x);
  return ret;
}

// config detail: {'bit': 0, 'enum': {0: 'PMF_EPB_ONOFF_OFF', 1:
// 'PMF_EPB_ONOFF_ON'}, 'is_signed_var': False, 'len': 4, 'name':
// 'pmf_epb_onoff', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Parkingmode_feedback_18ff8dac::Pmf_epb_onoffType
Parkingmodefeedback18ff8dac::pmf_epb_onoff(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  Parkingmode_feedback_18ff8dac::Pmf_epb_onoffType ret =
      static_cast<Parkingmode_feedback_18ff8dac::Pmf_epb_onoffType>(x);
  return ret;
}
}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
