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

#include "modules/planning/open_space/open_space_trajectory_generator.h"

#include <cmath>
#include <fstream>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::planning_internal::Trajectories;

Status OpenSpaceTrajectoryGenerator::Init(
    const PlannerOpenSpaceConfig& planner_open_space_config) {
  // nominal sampling time
  ts_ = planner_open_space_config.delta_t();

  // load vehicle configuration
  double front_to_center = vehicle_param_.front_edge_to_center();
  double back_to_center = vehicle_param_.back_edge_to_center();
  double left_to_center = vehicle_param_.left_edge_to_center();
  double right_to_center = vehicle_param_.right_edge_to_center();
  ego_.resize(4, 1);
  ego_ << front_to_center, right_to_center, back_to_center, left_to_center;

  // initialize warm start class pointer
  warm_start_.reset(new HybridAStar(planner_open_space_config));

  // initialize dual variable warm start class pointer
  dual_variable_warm_start_.reset(
      new DualVariableWarmStartProblem(planner_open_space_config));

  // initialize distance approach class pointer
  distance_approach_.reset(
      new DistanceApproachProblem(planner_open_space_config));
  return Status::OK();
}

apollo::common::Status OpenSpaceTrajectoryGenerator::Plan(
    const VehicleState& vehicle_state, const std::vector<double>& XYbounds,
    const double rotate_angle, const Vec2d& translate_origin,
    const std::vector<double>& end_pose, size_t obstacles_num,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    ThreadSafeIndexedObstacles* obstalce_list) {
  if (!vehicle_state.has_x() || XYbounds.size() == 0 || end_pose.size() == 0 ||
      obstacles_edges_num.cols() == 0 || obstacles_A.cols() == 0 ||
      obstacles_b.cols() == 0) {
    return Status(ErrorCode::PLANNING_ERROR, "Generator input data not ready");
  }

  // initial state
  init_state_ = vehicle_state;
  init_x_ = init_state_.x();
  init_y_ = init_state_.y();
  init_phi_ = init_state_.heading();
  // TODO(Jinyun) workaround the initial small speed to avoid problem in
  // trajectory partition
  init_v_ = 0.0;
  // rotate and scale the state according to the origin point defined in
  // frame
  init_x_ -= translate_origin.x();
  init_y_ -= translate_origin.y();
  double tmp_x = init_x_;
  init_x_ =
      init_x_ * std::cos(-rotate_angle) - init_y_ * std::sin(-rotate_angle);
  init_y_ = tmp_x * std::sin(-rotate_angle) + init_y_ * std::cos(-rotate_angle);
  init_phi_ = common::math::NormalizeAngle(init_phi_ - rotate_angle);
  // TODO(Jinyun) how to initial input not decided yet
  init_steer_ = 0.0;
  init_a_ = 0.0;
  Eigen::MatrixXd x0(4, 1);
  x0 << init_x_, init_y_, init_phi_, init_v_;

  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer_, init_a_;

  // final state
  Eigen::MatrixXd xF(4, 1);
  xF << end_pose[0], end_pose[1], end_pose[2], end_pose[3];

  // planning bound
  XYbounds_ = XYbounds;

  ADEBUG << "Start forming state warm start problem with configs setting : "
         << planner_open_space_config_.warm_start_config().ShortDebugString();

  // Warm Start (initial velocity is assumed to be 0 for now)
  Result result;

  if (warm_start_->Plan(x0(0, 0), x0(1, 0), x0(2, 0), xF(0, 0), xF(1, 0),
                        xF(2, 0), XYbounds_, obstalce_list, &result)) {
    ADEBUG << "State warm start problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "State warm start problem failed to solve");
  }
  // load Warm Start result(horizon is the "N", not the size of step points)
  horizon_ = result.x.size() - 1;
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon_);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.x.data(), horizon_ + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.y.data(), horizon_ + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.phi.data(), horizon_ + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.v.data(), horizon_ + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.steer.data(), horizon_);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result.a.data(), horizon_);
  xWS.row(0) = x;
  xWS.row(1) = y;
  xWS.row(2) = phi;
  xWS.row(3) = v;
  uWS.row(0) = steer;
  uWS.row(1) = a;

  // Step 8 : Formulate distance approach problem
  // solution from distance approach
  ADEBUG << "Start forming state warm start problem with configs setting : "
         << planner_open_space_config_.dual_variable_warm_start_config()
                .ShortDebugString();

  // result for distance approach problem
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;

  if (FLAGS_use_dual_variable_warm_start) {
    bool dual_variable_warm_start_status = dual_variable_warm_start_->Solve(
        horizon_, ts_, ego_, obstacles_num, obstacles_edges_num, obstacles_A,
        obstacles_b, xWS, &l_warm_up, &n_warm_up);

    if (dual_variable_warm_start_status) {
      ADEBUG << "Dual variable problem solved successfully!";
    } else {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Dual variable problem failed to solve");
    }
  } else {
    l_warm_up =
        0.5 * Eigen::MatrixXd::Ones(obstacles_edges_num.sum(), horizon_ + 1);
    n_warm_up = 0.5 * Eigen::MatrixXd::Ones(4 * obstacles_num, horizon_ + 1);
  }

  // Step 9 : Formulate distance approach problem
  // solution from distance approach
  ADEBUG << "Start Forming Distance approach problem with configs setting : "
         << planner_open_space_config_.distance_approach_config()
                .ShortDebugString();
  // result for distance approach problem
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;
  Eigen::MatrixXd dual_l_result_ds;
  Eigen::MatrixXd dual_n_result_ds;

  bool distance_approach_status = distance_approach_->Solve(
      x0, xF, last_time_u, horizon_, ts_, ego_, xWS, uWS, l_warm_up, n_warm_up,
      XYbounds_, obstacles_num, obstacles_edges_num, obstacles_A, obstacles_b,
      &state_result_ds, &control_result_ds, &time_result_ds, &dual_l_result_ds,
      &dual_n_result_ds);

  if (distance_approach_status) {
    ADEBUG << "Distance approach problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Distance approach problem failed to solve");
  }

  // record debug info
  if (FLAGS_enable_record_debug) {
    open_space_debug_.Clear();
    RecordDebugInfo(xWS, uWS, l_warm_up, n_warm_up, dual_l_result_ds,
                    dual_n_result_ds, state_result_ds, control_result_ds,
                    XYbounds_, obstalce_list);
  }
  // rescale the states to the world frame
  for (size_t i = 0; i < horizon_ + 1; i++) {
    double tmp_x = state_result_ds(0, i);
    state_result_ds(0, i) = state_result_ds(0, i) * std::cos(rotate_angle) -
                            state_result_ds(1, i) * std::sin(rotate_angle);
    state_result_ds(1, i) = tmp_x * std::sin(rotate_angle) +
                            state_result_ds(1, i) * std::cos(rotate_angle);
    state_result_ds(0, i) += translate_origin.x();
    state_result_ds(1, i) += translate_origin.y();
    state_result_ds(2, i) =
        common::math::NormalizeAngle(state_result_ds(2, i) + rotate_angle);
  }

  // Step 9 : Trajectory Partition and  Publish TrajectoryPoint
  // in planning trajectory. Result saved in trajectory_partition_.
  // Every time update, use trajectory_partition to store each ADCTrajectory
  Status trajectory_partition_status =
      TrajectoryPartition(state_result_ds, control_result_ds, time_result_ds);

  // record debug info
  if (trajectory_partition_status.ok() && FLAGS_enable_record_debug) {
    RecordDebugInfo();
  }

  return trajectory_partition_status;
}

void OpenSpaceTrajectoryGenerator::UpdateTrajectory(
    Trajectories* adc_trajectories,
    std::vector<canbus::Chassis::GearPosition>* gear_positions) {
  adc_trajectories->CopyFrom(trajectory_partition_);
  *gear_positions = gear_positions_;
}

void OpenSpaceTrajectoryGenerator::UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* open_space_debug) {
  open_space_debug->CopyFrom(open_space_debug_);
}

void OpenSpaceTrajectoryGenerator::RecordDebugInfo(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    const Eigen::MatrixXd& dual_l_result_ds,
    const Eigen::MatrixXd& dual_n_result_ds,
    const Eigen::MatrixXd& state_result_ds,
    const Eigen::MatrixXd& control_result_ds,
    const std::vector<double>& XYbounds,
    ThreadSafeIndexedObstacles* obstalce_list) {
  // load warm start trajectory
  auto* warm_start_trajectory =
      open_space_debug_.mutable_warm_start_trajectory();
  for (size_t i = 0; i < horizon_; i++) {
    auto* warm_start_point = warm_start_trajectory->add_vehicle_motion_point();
    warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_x(
        xWS(0, i));
    warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_y(
        xWS(1, i));
    warm_start_point->mutable_trajectory_point()
        ->mutable_path_point()
        ->set_theta(xWS(2, i));
    warm_start_point->mutable_trajectory_point()->set_v(xWS(3, i));
    warm_start_point->set_steer(uWS(0, i));
    warm_start_point->mutable_trajectory_point()->set_a(uWS(1, i));
  }
  auto* warm_start_point = warm_start_trajectory->add_vehicle_motion_point();
  warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_x(
      xWS(0, horizon_));
  warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_y(
      xWS(1, horizon_));
  warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
      xWS(2, horizon_));
  warm_start_point->mutable_trajectory_point()->set_v(xWS(3, horizon_));

  // load warm start dual variables
  size_t l_warm_up_cols = l_warm_up.rows();
  for (size_t i = 0; i < horizon_; i++) {
    for (size_t j = 0; j < l_warm_up_cols; j++) {
      open_space_debug_.add_warm_start_dual_lambda(l_warm_up(j, i));
    }
  }
  size_t n_warm_up_cols = n_warm_up.rows();
  for (size_t i = 0; i < horizon_; i++) {
    for (size_t j = 0; j < n_warm_up_cols; j++) {
      open_space_debug_.add_warm_start_dual_miu(n_warm_up(j, i));
    }
  }

  // load optimized dual variables
  size_t dual_l_result_ds_cols = dual_l_result_ds.rows();
  for (size_t i = 0; i < horizon_; i++) {
    for (size_t j = 0; j < dual_l_result_ds_cols; j++) {
      open_space_debug_.add_optimized_dual_lambda(dual_l_result_ds(j, i));
    }
  }
  size_t dual_n_result_ds_cols = dual_n_result_ds.rows();
  for (size_t i = 0; i < horizon_; i++) {
    for (size_t j = 0; j < dual_n_result_ds_cols; j++) {
      open_space_debug_.add_optimized_dual_miu(dual_n_result_ds(j, i));
    }
  }

  // load smoothed trajectory
  auto* smoothed_trajectory = open_space_debug_.mutable_smoothed_trajectory();
  for (size_t i = 0; i < horizon_; i++) {
    auto* smoothed_point = smoothed_trajectory->add_vehicle_motion_point();
    smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_x(
        state_result_ds(0, i));
    smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_y(
        state_result_ds(1, i));
    smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
        state_result_ds(2, i));
    smoothed_point->mutable_trajectory_point()->set_v(state_result_ds(3, i));
    smoothed_point->set_steer(control_result_ds(0, i));
    smoothed_point->mutable_trajectory_point()->set_a(control_result_ds(1, i));
  }
  auto* smoothed_point = smoothed_trajectory->add_vehicle_motion_point();
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_x(
      state_result_ds(0, horizon_));
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_y(
      state_result_ds(1, horizon_));
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
      state_result_ds(2, horizon_));
  smoothed_point->mutable_trajectory_point()->set_v(
      state_result_ds(3, horizon_));

  // load xy boundary (xmin, xmax, ymin, ymax)
  open_space_debug_.add_xy_boundary(XYbounds[0]);
  open_space_debug_.add_xy_boundary(XYbounds[1]);
  open_space_debug_.add_xy_boundary(XYbounds[2]);
  open_space_debug_.add_xy_boundary(XYbounds[3]);

  // load obstacles
  for (const auto& obstacle_box : (*obstalce_list).Items()) {
    auto* obstacle_ptr = open_space_debug_.add_obstacles();
    std::vector<Vec2d> vertices =
        obstacle_box->PerceptionBoundingBox().GetAllCorners();
    size_t vertices_size = vertices.size();
    for (size_t i = 0; i < vertices_size; i++) {
      obstacle_ptr->add_vertices_x_coords(vertices[i].x());
      obstacle_ptr->add_vertices_y_coords(vertices[i].y());
    }
  }
}

void OpenSpaceTrajectoryGenerator::RecordDebugInfo() {
  // load trajectories by optimization and partition
  open_space_debug_.mutable_trajectories()->CopyFrom(trajectory_partition_);
}

Status OpenSpaceTrajectoryGenerator::TrajectoryPartition(
    const Eigen::MatrixXd& state_result_ds,
    const Eigen::MatrixXd& control_result_ds,
    const Eigen::MatrixXd& time_result_ds) {
  double relative_time = 0.0;
  double distance_s = 0.0;

  Trajectories trajectory_partition;
  gear_positions_.clear();

  ::apollo::common::Trajectory* current_trajectory =
      trajectory_partition.add_trajectory();
  // set initial gear position for first ADCTrajectory depending on v
  // and check potential edge cases
  const size_t initial_gear_check_horizon = 3;
  const double kepsilon = 1e-2;
  if (horizon_ < initial_gear_check_horizon)
    return Status(ErrorCode::PLANNING_ERROR, "Invalid trajectory length!");
  int direction_flag = 0;
  size_t i = 0;
  int j = 0;
  int init_direction = 0;
  while (i != initial_gear_check_horizon) {
    if (state_result_ds(3, j) > kepsilon) {
      i++;
      j++;
      direction_flag++;
      if (init_direction == 0) {
        init_direction++;
      }
    } else if (state_result_ds(3, j) < -kepsilon) {
      i++;
      j++;
      direction_flag--;
      if (init_direction == 0) {
        init_direction--;
      }
    } else {
      j++;
    }
  }
  if (direction_flag > 1) {
    gear_positions_.push_back(canbus::Chassis::GEAR_DRIVE);
  } else if (direction_flag < -1) {
    gear_positions_.push_back(canbus::Chassis::GEAR_REVERSE);
  } else {
    if (init_direction > 0) {
      ADEBUG << "initial speed oscillate too "
                "frequent around zero";
      gear_positions_.push_back(canbus::Chassis::GEAR_DRIVE);
    } else if (init_direction < 0) {
      ADEBUG << "initial speed oscillate too "
                "frequent around zero";
      gear_positions_.push_back(canbus::Chassis::GEAR_REVERSE);
    } else {
      return Status(
          ErrorCode::PLANNING_ERROR,
          "Invalid trajectory start! initial speeds too small to decide gear");
    }
  }
  // partition trajectory points into each trajectory
  for (size_t i = 0; i < horizon_ + 1; i++) {
    // shift from GEAR_DRIVE to GEAR_REVERSE if v < 0
    // then add a new trajectory with GEAR_REVERSE
    if (state_result_ds(3, i) < -kepsilon &&
        gear_positions_.back() == canbus::Chassis::GEAR_DRIVE) {
      current_trajectory = trajectory_partition.add_trajectory();
      gear_positions_.push_back(canbus::Chassis::GEAR_REVERSE);
      distance_s = 0.0;
      relative_time = 0.0;
    }
    // shift from GEAR_REVERSE to GEAR_DRIVE if v > 0
    // then add a new trajectory with GEAR_DRIVE
    if (state_result_ds(3, i) > kepsilon &&
        gear_positions_.back() == canbus::Chassis::GEAR_REVERSE) {
      current_trajectory = trajectory_partition.add_trajectory();
      gear_positions_.push_back(canbus::Chassis::GEAR_DRIVE);
      distance_s = 0.0;
    }

    auto* point = current_trajectory->add_trajectory_point();
    point->set_relative_time(relative_time);
    relative_time += time_result_ds(0, i);
    point->mutable_path_point()->set_x(state_result_ds(0, i));
    point->mutable_path_point()->set_y(state_result_ds(1, i));
    point->mutable_path_point()->set_theta(state_result_ds(2, i));
    if (i > 0) {
      distance_s +=
          std::sqrt((state_result_ds(0, i) - state_result_ds(0, i - 1)) *
                        (state_result_ds(0, i) - state_result_ds(0, i - 1)) +
                    (state_result_ds(1, i) - state_result_ds(1, i - 1)) *
                        (state_result_ds(1, i) - state_result_ds(1, i - 1)));
    }
    point->mutable_path_point()->set_s(distance_s);
    int gear_drive = 1;
    if (gear_positions_.back() == canbus::Chassis::GEAR_REVERSE)
      gear_drive = -1;

    point->set_v(state_result_ds(3, i) * gear_drive);
    // TODO(Jiaxuan): Verify this steering to kappa equation
    point->mutable_path_point()->set_kappa(
        std::tanh(control_result_ds(0, i) * 470 * M_PI / 180.0 / 16) / 2.85 *
        gear_drive);
    point->set_a(control_result_ds(1, i) * gear_drive);
  }

  trajectory_partition_.CopyFrom(trajectory_partition);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
