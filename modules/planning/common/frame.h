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

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/indexed_queue.h"
// #include "modules/planning/common/lag_prediction.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

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
                 ReferenceLineProvider *reference_line_provider);

  const common::TrajectoryPoint &PlanningStartPoint() const;

  void InitData(const LocalView &local_view,
                const common::TrajectoryPoint &planning_start_point,
                const double start_time,
                const common::VehicleState &vehicle_state,
                ReferenceLineProvider *reference_line_provider);
  common::Status Init(
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints);

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  const std::list<ReferenceLineInfo> &reference_line_info() const;
  std::list<ReferenceLineInfo> *mutable_reference_line_info();

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

  const bool is_near_destination() const { return is_near_destination_; }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t> &id_to_priority);

  const LocalView &local_view() const { return local_view_; }

  ThreadSafeIndexedObstacles *GetObstacleList() { return &obstacles_; }

  // @brief return the obstacle list for warm start in open space planner
  ThreadSafeIndexedObstacles *GetOSWSObstacleList() {
    return &openspace_warmstart_obstacles_;
  }

  const std::size_t obstacles_num() { return obstacles_num_; }

  const Eigen::MatrixXd &obstacles_edges_num() { return obstacles_edges_num_; }

  const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec() {
    return obstacles_vertices_vec_;
  }

  const Eigen::MatrixXd &obstacles_A() { return obstacles_A_; }

  const Eigen::MatrixXd &obstacles_b() { return obstacles_b_; }

  const double origin_heading() { return origin_heading_; }

  const Vec2d &origin_point() { return origin_point_; }

  ThreadSafeIndexedObstacles *openspace_warmstart_obstacles() {
    return &openspace_warmstart_obstacles_;
  }
  const std::vector<double> &ROI_xy_boundary() { return ROI_xy_boundary_; }

  const std::vector<double> &open_space_end_pose() {
    return open_space_end_pose_;
  }

 private:
  // @brief "Region of Interest", load open space xy boundary and parking space
  // boundary from pnc map (only for T shape parking space)
  // to ROI_xy_boundary_ and ROI_parking_boundary_
  bool OpenSpaceROI();

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
  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;
  ChangeLaneDecider change_lane_decider_;
  ADCTrajectory trajectory_;  // last published trajectory

  // TODO(All): add lag_predictor back
  // std::unique_ptr<LagPrediction> lag_predictor_;
  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

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
  // the viewing angle and the boundaries names
  //
  //                     ------------------------   <-  up_boundary
  //
  //  left_boundary  |-> --------      ----------   <-|  right_boundary
  //                 |->        |      |            <-|
  //                            |      |
  //                            |      |
  //                            |------|
  //                                ^
  //                          down_boundary
  ThreadSafeIndexedObstacles openspace_warmstart_obstacles_;

  // @brief in the order of [x_min, x_max, y_min, y_max];
  std::vector<double> ROI_xy_boundary_;

  // @brief vectors in the order of left parking bound, parking end bound, right
  // parking bound, opposite lane. And the vertices order is counter-clockwise
  // the viewing angle and the vertice sequence
  //
  //                     8------------------------7   <-  up_boundary
  //
  //  left_boundary  |-> 1-------2      5----------6   <-|  right_boundary
  //                 |->         |      |            <-|
  //                             |      |
  //                             |      |
  //                            |3------4|
  //                                ^
  //                          down_boundary
  // ROI_parking_boundary_ in form of {{1,2,3},{3,4},{4,5,6},{7,8}}
  std::vector<std::vector<Vec2d>> ROI_parking_boundary_;

  // @brief open_space end configuration in order of x, y, heading and speed.
  // Speed is set to be always zero now for parking
  std::vector<double> open_space_end_pose_;

  // @brief vector storing the vertices of obstacles in counter-clock-wise order
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec_;

  // @brief Linear inequality representation of the obstacles Ax>b
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;

  // @brief origin heading for planning space rotation
  double origin_heading_;

  // @brief origin point for scaling down the numeric value of the optimization
  // problem in order of x , y
  Vec2d origin_point_;
  bool init_data_ = false;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo
