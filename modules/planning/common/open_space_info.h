/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "cyber/common/log.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/proto/planning_internal.pb.h"

namespace apollo {
namespace planning {

typedef std::pair<DiscretizedTrajectory, canbus::Chassis::GearPosition>
    TrajGearPair;

struct GearSwitchStates {
  bool gear_switching_flag = false;
  bool gear_shift_period_finished = true;
  bool gear_shift_period_started = true;
  double gear_shift_period_time = 0.0;
  double gear_shift_start_time = 0.0;
  apollo::canbus::Chassis::GearPosition gear_shift_position =
      canbus::Chassis::GEAR_DRIVE;
};

class OpenSpaceInfo {
 public:
  OpenSpaceInfo() = default;
  ~OpenSpaceInfo() = default;

  const std::string target_parking_spot_id() const {
    return target_parking_spot_id_;
  }

  std::string *mutable_target_parking_spot_id() {
    return &target_parking_spot_id_;
  }

  const hdmap::ParkingSpaceInfoConstPtr target_parking_spot() const {
    return target_parking_spot_;
  }

  hdmap::ParkingSpaceInfoConstPtr *mutable_target_parking_spot() {
    return &target_parking_spot_;
  }

  const hdmap::LaneInfoConstPtr target_parking_lane() const {
    return target_parking_lane_;
  }

  void set_target_parking_lane(hdmap::LaneInfoConstPtr lane_info_const_ptr) {
    target_parking_lane_ = lane_info_const_ptr;
  }

  double open_space_pre_stop_fence_s() const {
    return open_space_pre_stop_fence_s_;
  }

  void set_open_space_pre_stop_fence_s(const double s) {
    open_space_pre_stop_fence_s_ = s;
  }

  bool pre_stop_rightaway_flag() const { return pre_stop_rightaway_flag_; }

  void set_pre_stop_rightaway_flag(const bool flag) {
    pre_stop_rightaway_flag_ = flag;
  }

  const hdmap::MapPathPoint &pre_stop_rightaway_point() const {
    return pre_stop_rightaway_point_;
  }

  hdmap::MapPathPoint *mutable_pre_stop_rightaway_point() {
    return &pre_stop_rightaway_point_;
  }

  bool is_on_open_space_trajectory() const {
    return is_on_open_space_trajectory_;
  }

  void set_is_on_open_space_trajectory(const bool flag) {
    is_on_open_space_trajectory_ = flag;
  }

  size_t obstacles_num() const { return obstacles_num_; }

  void set_obstacles_num(const size_t obstacles_num) {
    obstacles_num_ = obstacles_num;
  }

  const Eigen::MatrixXi &obstacles_edges_num() const {
    return obstacles_edges_num_;
  }

  Eigen::MatrixXi *mutable_obstacles_edges_num() {
    return &obstacles_edges_num_;
  }

  const std::vector<std::vector<common::math::Vec2d>> &obstacles_vertices_vec()
      const {
    return obstacles_vertices_vec_;
  }

  std::vector<std::vector<common::math::Vec2d>>
      *mutable_obstacles_vertices_vec() {
    return &obstacles_vertices_vec_;
  }

  const Eigen::MatrixXd &obstacles_A() const { return obstacles_A_; }

  Eigen::MatrixXd *mutable_obstacles_A() { return &obstacles_A_; }

  const Eigen::MatrixXd &obstacles_b() const { return obstacles_b_; }

  Eigen::MatrixXd *mutable_obstacles_b() { return &obstacles_b_; }

  double origin_heading() const { return origin_heading_; }

  void set_origin_heading(const double original_heading) {
    origin_heading_ = original_heading;
  }

  const common::math::Vec2d &origin_point() const { return origin_point_; }

  common::math::Vec2d *mutable_origin_point() { return &origin_point_; }

  const std::vector<double> &ROI_xy_boundary() const {
    return ROI_xy_boundary_;
  }

  std::vector<double> *mutable_ROI_xy_boundary() { return &ROI_xy_boundary_; }

  const std::vector<double> &open_space_end_pose() const {
    return open_space_end_pose_;
  }

  std::vector<double> *mutable_open_space_end_pose() {
    return &open_space_end_pose_;
  }

  const DiscretizedTrajectory &optimizer_trajectory_data() const {
    return optimizer_trajectory_data_;
  }

  DiscretizedTrajectory *mutable_optimizer_trajectory_data() {
    return &optimizer_trajectory_data_;
  }

  const std::vector<common::TrajectoryPoint> &stitching_trajectory_data()
      const {
    return stitching_trajectory_data_;
  }

  std::vector<common::TrajectoryPoint> *mutable_stitching_trajectory_data() {
    return &stitching_trajectory_data_;
  }

  const DiscretizedTrajectory &stitched_trajectory_result() const {
    return stitched_trajectory_result_;
  }

  DiscretizedTrajectory *mutable_stitched_trajectory_result() {
    return &stitched_trajectory_result_;
  }

  bool open_space_provider_success() const {
    return open_space_provider_success_;
  }

  void set_open_space_provider_success(const bool flag) {
    open_space_provider_success_ = flag;
  }

  bool destination_reached() const { return destination_reached_; }

  void set_destination_reached(const bool flag) { destination_reached_ = flag; }

  const DiscretizedTrajectory &interpolated_trajectory_result() const {
    return interpolated_trajectory_result_;
  }

  DiscretizedTrajectory *mutable_interpolated_trajectory_result() {
    return &interpolated_trajectory_result_;
  }

  const std::vector<TrajGearPair> &partitioned_trajectories() const {
    // TODO(Runxin): export to chart
    return partitioned_trajectories_;
  }

  std::vector<TrajGearPair> *mutable_partitioned_trajectories() {
    return &partitioned_trajectories_;
  }

  const GearSwitchStates &gear_switch_states() const {
    return gear_switch_states_;
  }

  GearSwitchStates *mutable_gear_switch_states() {
    return &gear_switch_states_;
  }

  const TrajGearPair &chosen_partitioned_trajectory() const {
    // TODO(Runxin): export to chart
    return chosen_partitioned_trajectory_;
  }

  TrajGearPair *mutable_chosen_partitioned_trajectory() {
    return &chosen_partitioned_trajectory_;
  }

  bool fallback_flag() const { return fallback_flag_; }

  void set_fallback_flag(const bool flag) { fallback_flag_ = flag; }

  TrajGearPair *mutable_fallback_trajectory() { return &fallback_trajectory_; }

  const TrajGearPair &fallback_trajectory() const {
    return fallback_trajectory_;
  }

  void set_fallback_trajectory(const TrajGearPair &traj_gear_pair) {
    fallback_trajectory_ = traj_gear_pair;
  }

  std::pair<PublishableTrajectory, canbus::Chassis::GearPosition>
      *mutable_publishable_trajectory_data() {
    return &publishable_trajectory_data_;
  }

  const std::pair<PublishableTrajectory, canbus::Chassis::GearPosition>
      &publishable_trajectory_data() const {
    return publishable_trajectory_data_;
  }

  // TODO(QiL, Jinyun) refactor and merge this with debug
  common::TrajectoryPoint *mutable_future_collision_point() {
    return &future_collision_point_;
  }

  const common::TrajectoryPoint &future_collision_point() const {
    return future_collision_point_;
  }

  // TODO(QiL, Jinyun): refactor open_space_info vs debug

  apollo::planning_internal::Debug *mutable_debug() { return debug_; }

  void set_debug(apollo::planning_internal::Debug *debug) { debug_ = debug; }

  const apollo::planning_internal::Debug &debug() const { return *debug_; }

  const apollo::planning_internal::Debug debug_instance() const {
    return debug_instance_;
  }

  apollo::planning_internal::Debug *mutable_debug_instance() {
    return &debug_instance_;
  }

  void sync_debug_instance() {
    // Remove existing obstacle vectors to prevent repeating obstacle
    // vectors.
    if (!debug_->planning_data().open_space().obstacles().empty()) {
      debug_instance_.mutable_planning_data()
          ->mutable_open_space()
          ->clear_obstacles();
    }
    debug_instance_.MergeFrom(*debug_);
  }

  void RecordDebug(apollo::planning_internal::Debug *ptr_debug);

  void set_time_latency(double time_latency) { time_latency_ = time_latency; }

 private:
  std::string target_parking_spot_id_;

  hdmap::ParkingSpaceInfoConstPtr target_parking_spot_ = nullptr;

  hdmap::LaneInfoConstPtr target_parking_lane_ = nullptr;

  double open_space_pre_stop_fence_s_ = 0.0;

  bool pre_stop_rightaway_flag_ = false;

  hdmap::MapPathPoint pre_stop_rightaway_point_;

  bool is_on_open_space_trajectory_ = false;
  // @brief obstacles total num including perception obstacles and parking space
  // boundary
  size_t obstacles_num_ = 0;

  // @brief the dimension needed for A and b matrix dimension in H
  // representation
  Eigen::MatrixXi obstacles_edges_num_;

  // @brief in the order of [x_min, x_max, y_min, y_max];
  std::vector<double> ROI_xy_boundary_;

  // @brief open_space end configuration in order of x, y, heading and speed.
  // Speed is set to be always zero now for parking
  std::vector<double> open_space_end_pose_;

  // @brief vector storing the vertices of obstacles in counter-clock-wise order
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;

  // @brief Linear inequality representation of the obstacles Ax>b
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;

  // @brief origin heading for planning space rotation
  double origin_heading_ = 0.0;

  // @brief origin point for scaling down the numeric value of the optimization
  // problem in order of x , y
  common::math::Vec2d origin_point_;

  DiscretizedTrajectory optimizer_trajectory_data_;

  std::vector<common::TrajectoryPoint> stitching_trajectory_data_;

  DiscretizedTrajectory stitched_trajectory_result_;

  bool open_space_provider_success_ = false;

  bool destination_reached_ = false;

  DiscretizedTrajectory interpolated_trajectory_result_;

  std::vector<TrajGearPair> partitioned_trajectories_;

  GearSwitchStates gear_switch_states_;

  TrajGearPair chosen_partitioned_trajectory_;

  bool fallback_flag_ = true;

  TrajGearPair fallback_trajectory_;

  common::TrajectoryPoint future_collision_point_;

  std::pair<PublishableTrajectory, canbus::Chassis::GearPosition>
      publishable_trajectory_data_;

  // the pointer from ADCtrajectory
  apollo::planning_internal::Debug *debug_;

  // the instance inside debug,
  // if ADCtrajectory is NULL, blank; else same to ADCtrajectory
  apollo::planning_internal::Debug debug_instance_;

  double time_latency_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
