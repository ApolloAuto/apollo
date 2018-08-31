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

#ifndef MODULES_PLANNING_PLANNING_BASE_H_
#define MODULES_PLANNING_PLANNING_BASE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ctpl/ctpl_stl.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase : public apollo::common::ApolloApp {
 public:
  PlanningBase() = default;
  virtual ~PlanningBase();

  virtual void RunOnce() = 0;

  // Watch dog timer
  virtual void OnTimer(const ros::TimerEvent&) = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* trajectory) = 0;

 protected:
  void PublishPlanningPb(ADCTrajectory* trajectory_pb, double timestamp);

  /**
   * @brief Fill the header and publish the planning message.
   */
  void Publish(planning::ADCTrajectory* trajectory) {
    using apollo::common::adapter::AdapterManager;
    AdapterManager::FillPlanningHeader(Name(), trajectory);
    AdapterManager::PublishPlanning(*trajectory);
  }

  void RegisterPlanners();

  bool IsVehicleStateValid(const common::VehicleState& vehicle_state);

  virtual void SetFallbackTrajectory(ADCTrajectory* cruise_trajectory);

  void CheckPlanningConfig();

  double start_time_ = 0.0;

  common::util::Factory<PlanningConfig::PlannerType, Planner> planner_factory_;

  PlanningConfig config_;

  TrafficRuleConfigs traffic_rule_configs_;

  const hdmap::HDMap* hdmap_ = nullptr;

  std::unique_ptr<Planner> planner_;

  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;

  ros::Timer timer_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNING_BASE_H_
