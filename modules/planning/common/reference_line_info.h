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

/**
 * @file
 **/

#ifndef MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
#define MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

class ReferenceLineInfo {
 public:
  explicit ReferenceLineInfo(const common::VehicleState& vehicle_state,
                             const common::TrajectoryPoint& adc_planning_point,
                             const ReferenceLine& reference_line,
                             const hdmap::RouteSegments& segments);

  bool Init();

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  PathObstacle* AddObstacle(const Obstacle* obstacle);

  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  const ReferenceLine& reference_line() const;
  const common::TrajectoryPoint& AdcPlanningPoint() const;

  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);

  const DiscretizedTrajectory& trajectory() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  planning_internal::Debug* mutable_debug() { return &debug_; }
  const planning_internal::Debug& debug() const { return debug_; }
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  SpeedData* mutable_speed_data();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      const double relative_time,
      DiscretizedTrajectory* discretized_trajectory);

  const SLBoundary& AdcSlBoundary() const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const;
  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDriable(bool drivable);
  bool IsDrivable() const;

  const hdmap::RouteSegments& Lanes() const;

  void ExportDecision(DecisionResult* decision_result) const;

 private:
  void ExportTurnSignal(common::VehicleSignal* signal) const;

  std::unique_ptr<PathObstacle> CreatePathObstacle(const Obstacle* obstacle);
  bool InitPerceptionSLBoundary(PathObstacle* path_obstacle);

  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  const ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = false;

  PathDecision path_decision_;

  PathData path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
