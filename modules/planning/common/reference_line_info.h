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

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  enum class PathDataType {
    REGULAR_PATH,
    FALLBACK_PATH,
    REFERENCE_LINE_PATH,
  };
  ReferenceLineInfo() = default;
  explicit ReferenceLineInfo(const common::VehicleState& vehicle_state,
                             const common::TrajectoryPoint& adc_planning_point,
                             const ReferenceLine& reference_line,
                             const hdmap::RouteSegments& segments);

  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool IsInited() const;

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  Obstacle* AddObstacle(const Obstacle* obstacle);

  const common::VehicleState& vehicle_state() { return vehicle_state_; }

  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  const ReferenceLine& reference_line() const;

  double SDistanceToDestination() const;
  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);

  const DiscretizedTrajectory& trajectory() const;
  double TrajectoryLength() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  void SetStopPoint(const StopPoint& stop_point);
  void SetCruiseSpeed(double speed);
  const PlanningTarget& planning_target() const { return planning_target_; }

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

  void SetFeasiblePathData(PathDataType tag) { feasible_path_data_ = tag; }
  PathDataType GetFeasiblePathData() { return feasible_path_data_; }
  const PathData& path_data() const;
  const PathData& fallback_path_data() const;
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  PathData* mutable_fallback_path_data();
  SpeedData* mutable_speed_data();

  const RSSInfo& rss_info() const;
  RSSInfo* mutable_rss_info();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  const SLBoundary& AdcSlBoundary() const;
  const SLBoundary& VehicleSlBoundary() const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const;

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  void ExportEngageAdvice(common::EngageAdvice* engage_advice) const;

  bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }

  const hdmap::RouteSegments& Lanes() const;
  const std::list<hdmap::Id> TargetLaneId() const;

  void ExportDecision(DecisionResult* decision_result) const;

  void SetJunctionRightOfWay(const double junction_s,
                             const bool is_protected) const;

  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  const hdmap::Lane::LaneTurn& GetPathTurnType() const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  void GetPathBoundaries(
      std::vector<std::pair<double, double>>* const ptr_path_boundaries,
      double* const ptr_start_s, double* const ptr_resolution_s) {
    *ptr_path_boundaries = path_boundaries_;
    *ptr_start_s = path_boundaries_s_start_;
    *ptr_resolution_s = path_boundaries_s_resolution_;
  }

  void SetPathBoundaries(
      const std::vector<std::pair<double, double>>& path_boundaries,
      const double start_s, const double resolution_s) {
    path_boundaries_ = path_boundaries;
    path_boundaries_s_start_ = start_s;
    path_boundaries_s_resolution_ = resolution_s;
  }

  std::string GetBlockingObstacleId() const { return blocking_obstacle_id_; }

  void GetFallbackPathBoundaries(
      std::vector<std::pair<double, double>>* const ptr_path_boundaries,
      double* const ptr_start_s, double* const ptr_resolution_s) {
    *ptr_path_boundaries = fallback_path_boundaries_;
    *ptr_start_s = fallback_path_boundaries_s_start_;
    *ptr_resolution_s = fallback_path_boundaries_s_resolution_;
  }

  void SetFallbackPathBoundaries(
      const std::vector<std::pair<double, double>>& path_boundaries,
      const double start_s, const double resolution_s) {
    fallback_path_boundaries_ = path_boundaries;
    fallback_path_boundaries_s_start_ = start_s;
    fallback_path_boundaries_s_resolution_ = resolution_s;
  }

  void SetBlockingObstacleId(const std::string& blocking_obstacle_id) {
    blocking_obstacle_id_ = blocking_obstacle_id;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  void InitFirstOverlaps();

  uint32_t GetPriority() const { return reference_line_.GetPriority(); }

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }

  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

  StGraphData* mutable_st_graph_data() { return &st_graph_data_; }

  const StGraphData& st_graph_data() { return st_graph_data_; }

  // different types of overlaps that can be handled by different scenarios.
  enum OverlapType {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
  };

  const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
  FirstEncounteredOverlaps() const {
    return first_encounter_overlaps_;
  }

  int GetPnCJunction(const double s,
                     hdmap::PathOverlap* pnc_junction_overlap) const;

 private:
  bool CheckChangeLane() const;

  void ExportTurnSignal(common::VehicleSignal* signal) const;

  bool IsUnrelaventObstacle(const Obstacle* obstacle);

  void MakeDecision(DecisionResult* decision_result) const;

  int MakeMainStopDecision(DecisionResult* decision_result) const;

  void MakeMainMissionCompleteDecision(DecisionResult* decision_result) const;

  void MakeEStopDecision(DecisionResult* decision_result) const;

  void SetObjectDecisions(ObjectDecisions* object_decisions) const;

  bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

  bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                       hdmap::PathOverlap* path_overlap);

  void SetPathTurnType();

 private:
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_inited_ = false;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  std::vector<std::pair<double, double>> path_boundaries_;
  double path_boundaries_s_start_ = 0.0;
  double path_boundaries_s_resolution_ = 0.1;
  std::string blocking_obstacle_id_ = "";

  std::vector<std::pair<double, double>> fallback_path_boundaries_;
  double fallback_path_boundaries_s_start_ = 0.0;
  double fallback_path_boundaries_s_resolution_ = 0.1;

  // 1: path_data_; 0: fallback_path_data_; -1: neither.
  PathDataType feasible_path_data_ = PathDataType::REGULAR_PATH;
  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  struct {
    /**
     * @brief SL boundary of stitching point (starting point of plan trajectory)
     * relative to the reference line
     */
    SLBoundary adc_sl_boundary_;
    /**
     * @brief SL boundary of vehicle realtime state relative to the reference
     * line
     */
    SLBoundary vehicle_sl_boundary_;
  } sl_boundary_info_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = false;

  bool is_safe_to_change_lane_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

  /**
   * Overlaps encountered in the first time along the reference line in front of
   * the vehicle
   */
  std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
      first_encounter_overlaps_;

  hdmap::Lane::LaneTurn path_turn_type_;

  /**
   * @brief Data generated by speed_bounds_decider for constructing st_graph for
   * different st optimizer
   */
  StGraphData st_graph_data_;

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace apollo
