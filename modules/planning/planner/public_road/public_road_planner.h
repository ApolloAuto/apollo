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

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/planner/planner.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/tasks/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class PublicRoadPlanner
 * @brief PublicRoadPlanner is an expectation maximization planner.
 */

class PublicRoadPlanner : public PlannerWithReferenceLine {
 public:
  /**
   * @brief Constructor
   */
  PublicRoadPlanner() = delete;

  explicit PublicRoadPlanner(
      const std::shared_ptr<DependencyInjector>& injector)
      : PlannerWithReferenceLine(injector) {}

  /**
   * @brief Destructor
   */
  virtual ~PublicRoadPlanner() = default;

  void Stop() override {}

  std::string Name() override { return "PUBLIC_ROAD"; }

  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;
};

}  // namespace planning
}  // namespace apollo
