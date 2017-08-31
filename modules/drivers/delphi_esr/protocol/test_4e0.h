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

/**
 * @file test_4E0.h
 * @brief the class of 4E0 (for delphi_esr)
 */

#ifndef MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_TEST_4E0_H
#define MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_TEST_4E0_H

#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/sensor_protocol_data.h"

/**
 * @namespace apollo::drivers::delphi_esr
 * @brief apollo::drivers::delphi_esr
 */

namespace apollo {
namespace drivers {
namespace delphi_esr {

using ::apollo::drivers::DelphiESR;

class Test4e0 : public SensorProtocolData<DelphiESR> {
 public:
  static const int ID;

  void Parse(const uint8_t *bytes, int32_t length,
             DelphiESR *delphi_esr) const override;

  // config detail: {'name': 'num_obstacles', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'f_type': 'value', 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit':
  // '""'}
  int scan_index(const uint8_t *bytes, int32_t length) const;
};

}  // namespace delphi_esr 
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_DELPHI_ESR_PROTOCOL_TEST_4E0_H
