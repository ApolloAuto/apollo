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

#include <list>
#include <map>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/indexed_queue.h"
// #include "modules/planning/common/lag_prediction.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory_info.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);
  explicit Frame(uint32_t sequence_num, const LocalView &local_view,
                 const common::TrajectoryPoint &planning_start_point,
                 const double start_time,
                 const common::VehicleState &vehicle_state,
                 ReferenceLineProvider *reference_line_provider,
                 ADCTrajectory *output_trajectory);

  explicit Frame(uint32_t sequence_num, const LocalView &local_view,
                 const common::TrajectoryPoint &planning_start_point,
                 const double start_time,
                 const common::VehicleState &vehicle_state,
                 ADCTrajectory *output_trajectory);

  const common::TrajectoryPoint &PlanningStartPoint() const;

  common::Status Init(
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints);

  common::Status InitForOpenSpace();

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  void RecordOpenSpacePlannerDebug(planning_internal::Debug *debug);

  const std::list<ReferenceLineInfo> &reference_line_info() const;
  std::list<ReferenceLineInfo> *mutable_reference_line_info();
  const std::list<TrajectoryInfo> &trajectory_info() const;
  std::list<TrajectoryInfo> *mutable_trajectory_info();

  Obstacle *Find(const std::string &id);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting();

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  ADCTrajectory *output_trajectory() { return output_trajectory_; }

  planning_internal::OpenSpaceDebug *mutable_open_space_debug() {
    return &open_space_debug_;
  }

  const planning_internal::OpenSpaceDebug &open_space_debug() {
    return open_space_debug_;
  }

  std::vector<common::TrajectoryPoint> *mutable_last_stitching_trajectory() {
    return &stitching_trajectory_;
  }

  const std::vector<common::TrajectoryPoint> &last_stitching_trajectory() {
    return stitching_trajectory_;
  }

  const bool is_near_destination() const { return is_near_destination_; }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t> &id_to_priority);

  const LocalView &local_view() const { return local_view_; }

  ThreadSafeIndexedObstacles *GetObstacleList() { return &obstacles_; }

 private:
  common::Status InitFrameData();

  bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
                               const std::list<hdmap::RouteSegments> &segments);

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

 private:
  uint32_t sequence_num_ = 0;
  LocalView local_view_;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  double start_time_ = 0.0;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;
  std::list<TrajectoryInfo> trajectory_info_;

  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;
  ChangeLaneDecider change_lane_decider_;
  ADCTrajectory trajectory_;  // last published trajectory

  // debug info for open space planner
  planning_internal::OpenSpaceDebug open_space_debug_;
  // stitching trajectory for open space planner
  std::vector<common::TrajectoryPoint> stitching_trajectory_;

  // TODO(all): change to use shared_ptr.
  // output trajectory pb
  ADCTrajectory *output_trajectory_ = nullptr;  // not owned

  // TODO(All): add lag_predictor back
  // std::unique_ptr<LagPrediction> lag_predictor_;
  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  bool init_data_ = false;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory)
};

}  // namespace planning
}  // namespace apollo
