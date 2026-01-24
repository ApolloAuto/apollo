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

#include "modules/planning/planning_base/common/smoothers/smoother.h"
#include "modules/planning/planning_component/planning_base.h"

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
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {}

  virtual ~OnLanePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);

  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      const double curr_timestamp) const;

  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  bool CheckPlanningConfig(const PlanningConfig& config);
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);
  void ExportFailedLaneChangeSTChart(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  void ExportOnLaneChart(const planning_internal::Debug& debug_info,
                         planning_internal::Debug* debug_chart);
  void ExportOpenSpaceChart(const planning_internal::Debug& debug_info,
                            const ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart);
  void AddOpenSpaceOptimizerResult(const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart);
  void AddPartitionedTrajectory(const planning_internal::Debug& debug_info,
                                planning_internal::Debug* debug_chart);

  void AddStitchSpeedProfile(planning_internal::Debug* debug_chart);

  void AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                         planning_internal::Debug* debug_chart);

  void AddPublishedAcceleration(const ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug);

  void AddFallbackTrajectory(const planning_internal::Debug& debug_info,
                             planning_internal::Debug* debug_chart);

  void AddADCDrivingStateInfo(planning_internal::Debug* const debug);

 private:
  PlanningCommand last_command_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
};

}  // namespace planning
}  // namespace apollo
