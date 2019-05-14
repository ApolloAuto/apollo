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
#include <string>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/dreamview/proto/chart.pb.h"
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
#include "modules/planning/common/frame.h"
#include "modules/planning/common/local_view.h"
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
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase {
 public:
  PlanningBase() = default;

  virtual ~PlanningBase();

  virtual apollo::common::Status Init(const PlanningConfig& config);

  virtual std::string Name() const = 0;

  virtual void RunOnce(const LocalView& local_view,
                       ADCTrajectory* const adc_trajectory) = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) = 0;

 protected:
  virtual void FillPlanningPb(const double timestamp,
                              ADCTrajectory* const trajectory_pb);

  LocalView local_view_;
  const hdmap::HDMap* hdmap_ = nullptr;

  double start_time_ = 0.0;
  size_t seq_num_ = 0;

  PlanningConfig config_;
  TrafficRuleConfigs traffic_rule_configs_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::unique_ptr<PlannerDispatcher> planner_dispatcher_;
};

}  // namespace planning
}  // namespace apollo
