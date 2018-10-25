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

/**
 * @file
 **/

#pragma once

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Eigen"
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
#include "modules/common/util/util.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

using common::math::Vec2d;
/**
 * @class FrameOpenSpace
 *
 * @brief FrameOpenSpace holds all data for one planning cycle.
 */

class FrameOpenSpace {
 public:
  explicit FrameOpenSpace(uint32_t sequence_num, const LocalView &local_view,
                          const common::TrajectoryPoint &planning_start_point,
                          const double start_time,
                          const common::VehicleState &vehicle_state);

  common::Status Init();

  bool LoadDataOpenSpace();

  const common::TrajectoryPoint &PlanningStartPoint() const;

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  PathObstacle *Find(const std::string &id);

  const std::vector<const PathObstacle *> obstacles() const;

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  const bool is_near_destination() const { return is_near_destination_; }

  ThreadSafeIndexedObstacles *GetObstacleList() { return &obstacles_; }

  // @brief return the obstacle list for warm start in open space planner
  ThreadSafeIndexedObstacles *GetOSWSObstacleList() {
    return &openspace_warmstart_obstacles_;
  }

  const std::size_t obstacles_num() { return obstacles_num_; }

  const Eigen::MatrixXd &obstacles_edges_num() { return obstacles_edges_num_; }

  const std::vector<std::vector<Vec2d>> obstacles_vertices_vec() {
    return obstacles_vertices_vec_;
  }

  const Eigen::MatrixXd &obstacles_A() { return obstacles_A_; }

  const Eigen::MatrixXd &obstacles_b() { return obstacles_b_; }

  // TODO(Jinyun) : depends on mapping and pnc map
  // @brief "Region of Interest", load open space xy boundary and parking space
  // boundary from pnc map
  // to ROI_xy_boundary_ and ROI_parking_boundary_
  bool ROI() {
    // 1. get xy_boundary and and assemble parking space boundary for warm start
    // and distance approach
    // 2. check if two parking space boundary is the same size and zero
    // parking_boundaries_num = ROI_warmstart_parking_boundary_.size();
    // if (parking_boundaries_num == 0) {
    //   AINFO << "no obstacle by given by ROI";
    //   return false;
    // }
    return true;
  }

  // @brief Transform the vertice presentation of the obstacles into linear
  // inequality as Ax>b
  bool HPresentationObstacle();

  // @brief Helper function for HPresentationObstacle()
  bool ObsHRep(const std::size_t &obstacles_num,
               const Eigen::MatrixXd &obstacles_edges_num,
               const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
               Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);

  // @brief Represent the obstacles in vertices and load it into
  // obstacles_vertices_vec_ in clock wise order. Take different approach
  // towards warm start and distance approach
  bool VPresentationObstacle();

 private:
  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const PathObstacle *FindCollisionObstacle() const;

  void AddObstacle(const PathObstacle &obstacle);

  const LocalView &local_view() const { return local_view_; }

 private:
  uint32_t sequence_num_ = 0;
  const hdmap::HDMap *hdmap_ = nullptr;
  const LocalView local_view_;
  common::TrajectoryPoint planning_start_point_;
  const double start_time_;
  common::VehicleState vehicle_state_;
  bool is_near_destination_ = false;
  prediction::PredictionObstacles prediction_;
  ThreadSafeIndexedObstacles obstacles_;
  ChangeLaneDecider change_lane_decider_;
  ADCTrajectory trajectory_;  // last published trajectory
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  // @brief obstacles total num including perception obstacles and parking space
  // boundary
  std::size_t obstacles_num_ = 0;

  // @brief the dimension needed for A and b matrix dimension in H
  // representation
  Eigen::MatrixXd obstacles_edges_num_;

  // @brief obstacle list for open space warm start as warm start needs all
  // obstacles in shape of box while distance approach only requires lines for
  // parking boundary
  ThreadSafeIndexedObstacles openspace_warmstart_obstacles_;

  // @brief in the order of [x_min, x_max, y_min, y_max];
  std::vector<double> ROI_xy_boundary_;

  // @brief vectors in the order of opposite lane, left parking bound, right
  // parking bound (left or right decided as the car is facing backwards the
  // parking spot) and parking end bound
  std::vector<std::vector<double>> ROI_warmstart_parking_boundary_;

  // @brief similar to ROI_warmstart_parking_boundary_ but the difference in
  // these two is that the warm start requires the boundary to have a redundant
  // outward lines to formulate as a box to detect collision with while the
  // distance approach only needs the parking boundary lines, not need it to be
  // a close convex form
  // @brief the sequence of the vertice in the vector should be clock-wise with
  // the point starting from far left corner of left parking bound
  std::vector<std::vector<double>> ROI_distance_approach_parking_boundary_;

  // @brief vector storing the vertices of obstacles in clock-wise order
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec_;

  // @brief Linear inequality representation of the obstacles Ax>b
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;
};

class FrameOpenSpaceHistory : public IndexedQueue<uint32_t, FrameOpenSpace> {
 private:
  DECLARE_SINGLETON(FrameOpenSpaceHistory);
};

}  // namespace planning
}  // namespace apollo
