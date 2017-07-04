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

#ifndef MODULES_PLATFORM_HW_ESDCAN_CHECKER_H_
#define MODULES_PLATFORM_HW_ESDCAN_CHECKER_H_

#include <string>

#include "modules/monitor/common/annotations.h"
#include "modules/monitor/common/interface/hw_checker.h"

#include "esdcan_test.h"

/**
 * @namespace apollo::platform::hw
 * @brief apollo::platform::hw
 */
namespace apollo {
namespace platform {
namespace hw {

class EsdCanChecker : public HwCheckerIntf {
 public:
  static const std::string ESD_CAN_NAME;

  /// Returns a HW status code from ntstatus.
  static hw::Status esdcan_result_to_hw_status(NTCAN_RESULT ntstatus);

  /// Returns a HW status message from ntstatus.
  static std::string esdcan_result_to_message(NTCAN_RESULT ntstatus);

  explicit EsdCanChecker(int id);

  virtual ~EsdCanChecker() {}

  /// Returns the name of the HW this checker checks.
  const std::string &get_name() const override { return name_; };

  /// Runs HW status check, stores results in results.
  void run_check(std::vector<HwCheckResult> &results) override;

 private:
  int can_id_ = 0;
  std::string name_;

  DISALLOW_COPY_AND_ASSIGN(EsdCanChecker);
};

}  // namespace hw
}  // namespace platform
}  // namespace apollo

#endif  // MODULES_PLATFORM_INTERFACE_HW_CHECKER_H_
