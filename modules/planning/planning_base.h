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

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/planner/planner_dispatcher.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {
/**
 * @class planning_data
 *
 * @brief PlanningData contains all necessary data as planning input
 */

struct PlanningData {
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<routing::RoutingResponse> routing;
};

/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase {
 public:
  PlanningBase() = default;
  virtual ~PlanningBase();

  virtual apollo::common::Status Init() = 0;

  virtual std::string Name() const = 0;

  virtual void RunOnce(const PlanningData& planning_data,
                       ADCTrajectory* const adc_trajectory) = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) = 0;

 protected:
  void PublishPlanningPb(const double timestamp,
                         ADCTrajectory* const trajectory_pb);
  void SetFallbackTrajectory(ADCTrajectory* const trajectory_pb);

  PlanningData planning_data_;
  const hdmap::HDMap* hdmap_ = nullptr;
  const std::shared_ptr<ADCTrajectory> last_planning_;

  double start_time_ = 0.0;
  std::size_t seq_num_ = 0;

  PlanningConfig config_;
  TrafficRuleConfigs traffic_rule_configs_;
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::unique_ptr<PlannerDispatcher> planner_dispatcher_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNING_BASE_H_
