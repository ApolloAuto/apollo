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

#ifndef MODULES_PLANNING_PLANNING_H_
#define MODULES_PLANNING_PLANNING_H_

#include <memory>
#include <utility>
#include <vector>

#include "modules/common/util/factory.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/planner/planner.h"

namespace apollo {
namespace planning {

class Planning {
 public:
  /**
   * @brief Constructor
   */
  Planning();

  /**
   * @brief Destructor
   */
  ~Planning() = default;

  /**
   * @brief Plan the trajectory given current vehicle state
   * @param vehicle_state variable describes the vehicle state, including
   * position, velocity, acceleration, heading, etc
   * @param is_on_auto_mode whether the current system is on auto-driving mode
   * @param publishable_trajectory the computed planning trajectory
   */
  bool Plan(const common::vehicle_state::VehicleState &vehicle_state,
            const bool is_on_auto_mode, const double publish_time,
            std::vector<common::TrajectoryPoint> *discretized_trajectory);

  /**
   * @brief Reset the planner to initial state.
   */
  void Reset();

 private:
  void RegisterPlanners();

 private:
  std::pair<common::TrajectoryPoint, std::size_t>
  ComputeStartingPointFromLastTrajectory(const double curr_time) const;

  common::TrajectoryPoint ComputeStartingPointFromVehicleState(
      const common::vehicle_state::VehicleState &vehicle_state,
      const double forward_time) const;

  std::vector<common::TrajectoryPoint> GetOverheadTrajectory(
      const std::size_t matched_index, const std::size_t buffer_size);

  std::unique_ptr<Planner> ptr_planner_;

  enum PlannerType {
    RTK_PLANNER,
  };

  common::util::Factory<PlannerType, Planner> planner_factory_;

  std::vector<common::TrajectoryPoint> last_trajectory_;

  double last_header_time_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_H_ */
