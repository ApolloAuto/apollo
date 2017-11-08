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

#ifndef MODULES_MONITOR_HARDWARE_CAN_ESDCAN_ESDCAN_TEST_H_
#define MODULES_MONITOR_HARDWARE_CAN_ESDCAN_ESDCAN_TEST_H_

#include <ostream>
#include <string>

#include "esd_can/include/ntcan.h"
#include "modules/monitor/hardware/hardware_checker.h"

/**
 * @namespace apollo::monitor::hw
 * @brief apollo::monitor::hw
 */
namespace apollo {
namespace monitor {
namespace hw {

/// A collection of details data about a given ESD-CAN interface.
class EsdCanDetails : public ::apollo::monitor::HwCheckResultDetails {
 public:
  enum ValidMasks {
    IF_STATUS = 0x1u,
    STATS = 0x2u,
    CTRL_STATE = 0x4u,
    BITRATE = 0x8u
  };

  EsdCanDetails() : result(NTCAN_NET_NOT_FOUND), valid_flag(0) {}

  void print_summary(std::ostream &os) override;

  void print_test_result(std::ostream &os) override;

  /// Test (check) esdcan of the given id.
  /// @param stats where to store can bus stats.
  /// @param details where to store detailed can stats/state information.
  NTCAN_RESULT esdcan_do_test(int id);

  CAN_IF_STATUS if_status;
  NTCAN_BUS_STATISTIC stats;
  NTCAN_CTRL_STATE ctrl_state;
  NTCAN_BITRATE bitrate;

  NTCAN_RESULT result;
  /// Bits flag indicating which fields are valid.
  unsigned int valid_flag;

 private:
  /// Invalidates all fields.
  inline void invalidate() {
    result = NTCAN_NET_NOT_FOUND;
    valid_flag = 0;
  }

  /// Marks a specific field of the given mask as valid.
  inline void add_valid_field(ValidMasks mask) { valid_flag |= mask; }
};

}  // namespace hw
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_HARDWARE_CAN_ESDCAN_ESDCAN_TEST_H_
