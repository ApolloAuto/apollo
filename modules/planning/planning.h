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

#include "modules/common/apollo_app.h"
#include "modules/common/proto/path_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Localization
 *
 * @brief Localization module main class. It processes GPS and IMU as input,
 * to generate localization info.
 */
class Planning : public apollo::common::ApolloApp {
 public:
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  void Stop() override;

  /**
   * @brief Plan the trajectory given current vehicle state
   * @param vehicle_state variable describes the vehicle state, including
   * position,
   *        velocity, acceleration, heading, etc
   * @param is_on_auto_mode whether the current system is on auto-driving mode
   * @param publishable_trajectory the computed planning trajectory
   */
  bool Plan(const common::vehicle_state::VehicleState& vehicle_state,
            const bool is_on_auto_mode, const double publish_time,
            std::vector<common::TrajectoryPoint>* discretized_trajectory);

  /**
   * @brief Reset the planner to initial state.
   */
  void Reset();

 private:
  void RegisterPlanners();
  void RunOnce();

  std::pair<common::TrajectoryPoint, std::size_t>
  ComputeStartingPointFromLastTrajectory(const double curr_time) const;

  common::TrajectoryPoint ComputeStartingPointFromVehicleState(
      const common::vehicle_state::VehicleState& vehicle_state,
      const double forward_time) const;

  std::vector<common::TrajectoryPoint> GetOverheadTrajectory(
      const std::size_t matched_index, const std::size_t buffer_size);

  ADCTrajectory ToTrajectoryPb(
      const double header_time,
      const std::vector<common::TrajectoryPoint>& discretized_trajectory);


  apollo::common::util::Factory<PlanningConfig::PlannerType,
                                Planner>
      planner_factory_;
  PlanningConfig config_;

  std::unique_ptr<Planner> planner_;

  std::vector<common::TrajectoryPoint> last_trajectory_;

  double last_header_time_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_H_ */
