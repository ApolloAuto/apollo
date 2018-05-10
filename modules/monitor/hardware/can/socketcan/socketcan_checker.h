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

#ifndef MODULES_MONITOR_HARDWARE_CAN_SOCKETCAN_SOCKETCAN_CHECKER_H_
#define MODULES_MONITOR_HARDWARE_CAN_SOCKETCAN_SOCKETCAN_CHECKER_H_

#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/monitor/hardware/annotations.h"
#include "modules/monitor/hardware/can/socketcan/socketcan_test.h"
#include "modules/monitor/hardware/hardware_checker.h"

/**
 * @namespace apollo::monitor::hw
 * @brief apollo::monitor::hw
 */
namespace apollo {
namespace monitor {
namespace hw {

class SocketCanChecker : public HwCheckerInterface {
 public:
  static const char SOCKET_CAN_NAME[];

  /// Returns a HW status code from socketcan_status.
  static HardwareStatus::Status socketcan_result_to_hw_status(
      int socketcan_status);

  /// Returns a HW status message from socketcan_status.
  static std::string socketcan_result_to_message(int socketcan_status);

  SocketCanChecker();

  virtual ~SocketCanChecker() {}

  /// Returns the name of the HW this checker checks.
  const std::string &get_name() const override {
    return name_;
  }

  // Returns the can id
  const int &get_id() const {
    return can_id_;
  }

  /// Runs HW status check, stores results in results.
  void run_check(std::vector<HwCheckResult> *results) override;

 private:
  int can_id_ = 0;
  std::string name_;

  DISALLOW_COPY_AND_ASSIGN(SocketCanChecker);
};

}  // namespace hw
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_HARDWARE_CAN_SOCKETCAN_SOCKETCAN_CHECKER_H_
