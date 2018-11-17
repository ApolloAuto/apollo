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

#ifndef MODULES_MONITOR_HARDWARE_CAN_CAN_CHECKER_FACTORY_H_
#define MODULES_MONITOR_HARDWARE_CAN_CAN_CHECKER_FACTORY_H_

#include <memory>

#include "modules/common/macro.h"
#include "modules/common/util/factory.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/monitor/hardware/hardware_checker.h"

/**
 * @file: can_checker_factory.h
 */

namespace apollo {
namespace monitor {

class CanCheckerFactory
    : public apollo::common::util::Factory<
          ::apollo::drivers::canbus::CANCardParameter::CANCardBrand,
          HwCheckerInterface> {
 public:
  /**
   * @brief Register can checkers
   */
  void RegisterCanCheckers();

  /**
   * @brief Create a pointer to a can checker
   * @param The parameter to create a can checker
   * @return A pointer to the created can checker
   */
  std::unique_ptr<HwCheckerInterface> CreateCanChecker(
      const ::apollo::drivers::canbus::CANCardParameter &parameter);

 private:
  DECLARE_SINGLETON(CanCheckerFactory);
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_HARDWARE_CAN_CAN_CHECKER_FACTORY_H_

