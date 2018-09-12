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

#ifndef MODULES_PLANNING_STD_PLANNING_H_
#define MODULES_PLANNING_STD_PLANNING_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/planner/std_planner_dispatcher.h"
#include "modules/planning/planning_base.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class StdPlanning : public PlanningBase {
 public:
  StdPlanning() {
    planner_dispatcher_ = std::make_unique<StdPlannerDispatcher>();
  }
  virtual ~StdPlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
  * @brief main logic of the planning module, runs periodically triggered by
  * timer.
  */
  void RunOnce(const std::shared_ptr<prediction::PredictionObstacles>&
                   prediction_obstacles,
               const std::shared_ptr<canbus::Chassis>& chassis,
               const std::shared_ptr<localization::LocalizationEstimate>&
                   localization_estimate,
               const perception::TrafficLightDetection,
               const routing::RoutingResponse& routing) override;

  apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* trajectory) override;

 private:
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const common::VehicleState& vehicle_state);
  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  bool CheckPlanningConfig();

 private:
  routing::RoutingResponse last_routing_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_STD_PLANNING_H_
