/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "modules/common/util/factory.h"
#include "modules/planning/planner/planner_dispatcher.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief PlannerDispatcher module main class.
 */
class NaviPlannerDispatcher final : public PlannerDispatcher {
 public:
  NaviPlannerDispatcher() = default;
  virtual ~NaviPlannerDispatcher() = default;

  std::unique_ptr<Planner> DispatchPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector) override;
};

}  // namespace planning
}  // namespace apollo
