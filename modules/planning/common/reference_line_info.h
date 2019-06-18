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

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_boundary.h"
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
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  explicit ReferenceLineInfo(const common::VehicleState& vehicle_state,
                             const common::TrajectoryPoint& adc_planning_point,
                             const ReferenceLine& reference_line,
                             const hdmap::RouteSegments& segments);

  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  Obstacle* AddObstacle(const Obstacle* obstacle);

  const common::VehicleState& vehicle_state() const { return vehicle_state_; }

  PathDecision* path_decision();
  const PathDecision& path_decision() const;

  const ReferenceLine& reference_line() const;
  ReferenceLine* mutable_reference_line();

  double SDistanceToDestination() const;
  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);
  const DiscretizedTrajectory& trajectory() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  void SetStopPoint(const StopPoint& stop_point);
  void SetCruiseSpeed(double speed);
  const PlanningTarget& planning_target() const { return planning_target_; }

  hdmap::LaneInfoConstPtr LocateLaneInfo(const double s) const;

  bool GetNeighborLaneInfo(const ReferenceLineInfo::LaneType lane_type,
                           const double s, hdmap::Id* ptr_lane_id,
                           double* ptr_lane_width) const;

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

  const hdmap::RouteSegments& Lanes() const;
  const std::list<hdmap::Id> TargetLaneId() const;

  void ExportDecision(DecisionResult* decision_result) const;

  void SetJunctionRightOfWay(const double junction_s,
                             const bool is_protected) const;

  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  const hdmap::Lane::LaneTurn GetPathTurnType(const double s) const;

  const bool GetIntersectionRightofWayStatus(
      const hdmap::PathOverlap& pnc_junction_overlap) const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;

  void SetCandidatePathBoundaries(
      std::vector<PathBoundary> candidate_path_boundaries);

  const std::vector<PathData>& GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathData> candidate_path_data);

  std::string GetBlockingObstacleId() const { return blocking_obstacle_id_; }

  void SetBlockingObstacleId(const std::string& blocking_obstacle_id) {
    blocking_obstacle_id_ = blocking_obstacle_id;
  }

  bool is_path_lane_borrow() const { return is_path_lane_borrow_; }
  void set_is_path_lane_borrow(bool is_path_lane_borrow) {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

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
  void InitFirstOverlaps();

  bool CheckChangeLane() const;

  void ExportTurnSignal(common::VehicleSignal* signal) const;

  bool IsIrrelevantObstacle(const Obstacle& obstacle);

  void MakeDecision(DecisionResult* decision_result) const;

  int MakeMainStopDecision(DecisionResult* decision_result) const;

  void MakeMainMissionCompleteDecision(DecisionResult* decision_result) const;

  void MakeEStopDecision(DecisionResult* decision_result) const;

  void SetObjectDecisions(ObjectDecisions* object_decisions) const;

  bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

  bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                       hdmap::PathOverlap* path_overlap);

 private:
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  std::string blocking_obstacle_id_ = "";

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = false;

  bool is_path_lane_borrow_ = false;

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

  /**
   * @brief Data generated by speed_bounds_decider for constructing st_graph for
   * different st optimizer
   */
  StGraphData st_graph_data_;

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace apollo
