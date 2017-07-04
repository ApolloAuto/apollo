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

#ifndef MODULES_PLATFORM_INTERFACE_HW_CHECKER_H_
#define MODULES_PLATFORM_INTERFACE_HW_CHECKER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/monitor/common/annotations.h"

#include "hw_status.h"

/**
 * @namespace apollo::platform
 * @brief apollo::platform
 */
namespace apollo {
namespace platform {

static const int HW_CLASS_UNDEF = 0;

struct HwCheckResult {
  /// Name of the HW component.
  std::string name;
  /// Status code defined in hw::Status.
  int status = 0;
  /// Specific status message for human consumption.
  std::string mssg;

  // @todo: it is confusing and doesn't support copy operation here;
  // consider to use a common base class with a deeep copy function.
  /// HW-specific details, may or may not be present.
  std::unique_ptr<void, void (*)(void *)> details;

  explicit HwCheckResult()
      : status(hw::Status::UNDEF), details(nullptr, [](void *) {}) {}

  HwCheckResult(const std::string &_name, int _status,
                const std::string &_mssg = std::string("OK"))
      : name(_name),
        status(_status),
        mssg(_mssg),
        details(nullptr, [](void *) {}) {}

  template <typename DetailType>
  HwCheckResult(const std::string &_name, int _status,
                DetailType *_details PTR_OWNER_XFR,
                const std::string &_mssg = std::string("OK"))
      : name(_name),
        status(_status),
        mssg(_mssg),
        details(_details, [](void *value) {
          DetailType *detail = static_cast<DetailType *>(value);
          delete detail;
        }) {}
};

class HwCheckerIntf {
 public:
  /// Returns HW class (one of pre-defined: CAN, Camera, ...).
  virtual const int get_class() const { return HW_CLASS_UNDEF; };

  /// Returns the name of the HW this checker will check (e.g., ESD_CAN).
  virtual const std::string &get_name() const = 0;

  /// Runs HW status check, stores results in results. We use a vector
  /// here because there may be multiple instances of a certain type
  /// of hw (e.g., cameras).
  virtual void run_check(std::vector<HwCheckResult> &results) = 0;
};

}  // namespace platform
}  // namespace apollo

#endif  // MODULES_PLATFORM_INTERFACE_HW_CHECKER_H_
