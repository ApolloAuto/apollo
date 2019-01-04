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

#include "modules/planning/common/frame.h"
#include "modules/planning/planner/std_planner_dispatcher.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/open_space/trajectory_partition/trajectory_partitioner.h"

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
class OpenSpacePlanning : public PlanningBase {
 public:
  OpenSpacePlanning() {
    planner_dispatcher_ = std::make_unique<StdPlannerDispatcher>();
  }
  virtual ~OpenSpacePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

  void AddOpenSpaceTrajectory(planning_internal::Debug* debug);

  void AddStitchSpeedProfile(planning_internal::Debug* debug);

  void AddPublishedSpeed(planning_internal::Debug* debug,
                         ADCTrajectory* const ptr_trajectory_pb);

  void AddPublishedAcceleration(planning_internal::Debug* debug,
                                ADCTrajectory* const ptr_trajectory_pb);

  void ExportOpenSpaceChart(planning_internal::Debug* debug,
                            ADCTrajectory* const ptr_trajectory_pb);

  void FillPlanningPb(const double timestamp,
                      ADCTrajectory* const ptr_trajectory_pb) override;

 private:
  apollo::common::Status InitFrame(
      const uint32_t sequence_num,
      const common::TrajectoryPoint& planning_start_point,
      const double start_time, const common::VehicleState& vehicle_state,
      ADCTrajectory* output_trajectory);
  bool CheckPlanningConfig(const PlanningConfig& config);

  bool IsCollisionFreeTrajectory(const ADCTrajectory& trajectory_pb);

  void BuildPredictedEnvironment(const std::vector<const Obstacle*>& obstacles);

  void GenerateStopTrajectory(ADCTrajectory* const ptr_trajectory_pb);

 private:
  routing::RoutingResponse last_routing_;
  std::unique_ptr<Frame> frame_;
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles_;
  std::unique_ptr<PublishableTrajectory> last_trajectory_;
  std::vector<common::TrajectoryPoint> last_stitching_trajectory_;
  planning_internal::OpenSpaceDebug last_open_space_debug_;
  bool last_trajectory_succeeded_ = false;
  std::unique_ptr<TrajectoryPartitioner> trajectory_partitioner_;
};

}  // namespace planning
}  // namespace apollo
