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

#ifndef MODULES_PLANNING_PLANNER_FACTORY_H_
#define MODULES_PLANNING_PLANNER_FACTORY_H_

#include "planner/planner.h"

#include <memory>

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class PlanningType
 * @brief PlanningType is a enum class used to specify a planner instance.
 */
enum class PlannerType { RTK_PLANNER, OTHER };

/**
 * @class PlannerFactory
 * @brief PlannerFactory is a creator class that
 *        creates an instance of a specific planner.
 */
class PlannerFactory {
 public:
  PlannerFactory() = delete;
  /**
   * @brief Generate a planner instance.
   * @param planner_type The specific type of planner,
   *        currently only supports PlannerType::RTK_PLANNER
   * @return A unique pointer pointing to the planner instance.
   */
  static std::unique_ptr<Planner> CreateInstance(
      const PlannerType& planner_type);
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNER_FACTORY_H_ */
