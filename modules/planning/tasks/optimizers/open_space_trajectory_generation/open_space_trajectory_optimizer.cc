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

#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_optimizer.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::VehicleState;

OpenSpaceTrajectoryOptimizer::OpenSpaceTrajectoryOptimizer(
    const OpenSpaceTrajectoryOptimizerConfig& config)
    : config_(config) {
  // Load config
  config_ = config;

  // Initialize hybrid astar class pointer
  warm_start_.reset(new HybridAStar(config.planner_open_space_config()));

  // Initialize dual variable warm start class pointer
  dual_variable_warm_start_.reset(
      new DualVariableWarmStartProblem(config.planner_open_space_config()));

  // Initialize distance approach trajectory smootherclass pointer
  distance_approach_.reset(
      new DistanceApproachProblem(config.planner_open_space_config()));
}

common::Status OpenSpaceTrajectoryOptimizer::Plan(
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
    double rotate_angle, const common::math::Vec2d& translate_origin,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  if (XYbounds.empty() || end_pose.empty() ||
      obstacles_edges_num.cols() == 0 || obstacles_A.cols() == 0 ||
      obstacles_b.cols() == 0) {
    ADEBUG << "OpenSpaceTrajectoryOptimizer input data not ready";
    return Status(ErrorCode::PLANNING_ERROR,
                  "OpenSpaceTrajectoryOptimizer input data not ready");
  }

  // Generate Stop trajectory if init point close to destination
  if (IsInitPointNearDestination(stitching_trajectory.back(), end_pose,
                                 rotate_angle, translate_origin)) {
    ADEBUG << "Planning init point is close to destination, skip new "
              "trajectory generation";
    return Status(ErrorCode::OK,
                  "Planning init point is close to destination, skip new "
                  "trajectory generation");
  }

  // Initiate initial states
  stitching_trajectory_ = stitching_trajectory;
  common::TrajectoryPoint init_trajectory_point = stitching_trajectory.back();
  common::PathPoint init_path_point = init_trajectory_point.path_point();
  double init_x = init_path_point.x();
  double init_y = init_path_point.y();
  double init_phi = init_path_point.theta();
  double init_v = init_trajectory_point.v();
  double init_steer = init_trajectory_point.steer();
  double init_a = init_trajectory_point.a();

  // Rotate and scale the state
  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                       &init_phi);

  Eigen::MatrixXd x0(4, 1);
  x0 << init_x, init_y, init_phi, init_v;

  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer, init_a;

  // Get final state
  Eigen::MatrixXd xF(4, 1);
  xF << end_pose[0], end_pose[1], end_pose[2], end_pose[3];

  // Result container for warm start (initial velocity is assumed to be 0 for
  // now)
  HybridAStartResult result;

  if (warm_start_->Plan(x0(0, 0), x0(1, 0), x0(2, 0), xF(0, 0), xF(1, 0),
                        xF(2, 0), XYbounds, obstacles_vertices_vec, &result)) {
    ADEBUG << "State warm start problem solved successfully!";
  } else {
    ADEBUG << "State warm start problem failed to solve";
    return Status(ErrorCode::PLANNING_ERROR,
                  "State warm start problem failed to solve");
  }
  // load Warm Start result(horizon is timestep number minus one)
  size_t horizon = result.x.size() - 1;
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result.a.data(), horizon);
  xWS.row(0) = x;
  xWS.row(1) = y;
  xWS.row(2) = phi;
  xWS.row(3) = v;
  uWS.row(0) = steer;
  uWS.row(1) = a;

  // Result container for distance approach problem
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;

  // load vehicle configuration
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double front_to_center = vehicle_param_.front_edge_to_center();
  double back_to_center = vehicle_param_.back_edge_to_center();
  double left_to_center = vehicle_param_.left_edge_to_center();
  double right_to_center = vehicle_param_.right_edge_to_center();
  Eigen::MatrixXd ego(4, 1);
  ego << front_to_center, right_to_center, back_to_center, left_to_center;

  // Get obstacle num
  size_t obstacles_num = obstacles_vertices_vec.size();

  // Get timestep delta t
  double ts = config_.planner_open_space_config().delta_t();

  if (FLAGS_use_dual_variable_warm_start) {
    if (dual_variable_warm_start_->Solve(
            horizon, ts, ego, obstacles_num, obstacles_edges_num, obstacles_A,
            obstacles_b, xWS, &l_warm_up, &n_warm_up)) {
      ADEBUG << "Dual variable problem solved successfully!";
    } else {
      ADEBUG << "Dual variable problem failed to solve";
      return Status(ErrorCode::PLANNING_ERROR,
                    "Dual variable problem failed to solve");
    }
  } else {
    l_warm_up =
        0.5 * Eigen::MatrixXd::Ones(obstacles_edges_num.sum(), horizon + 1);
    n_warm_up = 0.5 * Eigen::MatrixXd::Ones(4 * obstacles_num, horizon + 1);
  }

  // Result Containers for distance approach trajectory smoothing problem
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;
  Eigen::MatrixXd dual_l_result_ds;
  Eigen::MatrixXd dual_n_result_ds;

  if (distance_approach_->Solve(
          x0, xF, last_time_u, horizon, ts, ego, xWS, uWS, l_warm_up, n_warm_up,
          XYbounds, obstacles_num, obstacles_edges_num, obstacles_A,
          obstacles_b, &state_result_ds, &control_result_ds, &time_result_ds,
          &dual_l_result_ds, &dual_n_result_ds)) {
    ADEBUG << "Distance approach problem solved successfully!";
  } else {
    ADEBUG << "Distance approach problem failed to solve";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Distance approach problem failed to solve");
  }

  // rescale the states to the world frame
  for (size_t i = 0; i < horizon + 1; ++i) {
    PathPointDeNormalizing(rotate_angle, translate_origin,
                           &(state_result_ds(0, i)), &(state_result_ds(1, i)),
                           &(state_result_ds(2, i)));
  }

  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  return Status::OK();
}

bool OpenSpaceTrajectoryOptimizer::IsInitPointNearDestination(
    const common::TrajectoryPoint& planning_init_point,
    const std::vector<double>& end_pose, double rotate_angle,
    const Vec2d& translate_origin) {
  CHECK_EQ(end_pose.size(), 4);
  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  const common::PathPoint path_point = planning_init_point.path_point();
  double distance_to_init_point =
      std::sqrt((path_point.x() - end_pose_to_world_frame.x()) *
                    (path_point.x() - end_pose_to_world_frame.x()) +
                (path_point.y() - end_pose_to_world_frame.y()) *
                    (path_point.y() - end_pose_to_world_frame.y()));

  if (distance_to_init_point <
      config_.planner_open_space_config().is_near_destination_threshold()) {
    return true;
  }
  return false;
}

void OpenSpaceTrajectoryOptimizer::PathPointNormalizing(
    double rotate_angle, const common::math::Vec2d& translate_origin, double* x,
    double* y, double* phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void OpenSpaceTrajectoryOptimizer::PathPointDeNormalizing(
    double rotate_angle, const common::math::Vec2d& translate_origin, double* x,
    double* y, double* phi) {
  double tmp_x = *x;
  *x = (*x) * std::cos(rotate_angle) - (*y) * std::sin(rotate_angle);
  *y = tmp_x * std::sin(rotate_angle) + (*y) * std::cos(rotate_angle);
  *x += translate_origin.x();
  *y += translate_origin.y();
  *phi = common::math::NormalizeAngle(*phi + rotate_angle);
}

void OpenSpaceTrajectoryOptimizer::LoadTrajectory(
    const Eigen::MatrixXd& state_result, const Eigen::MatrixXd& control_result,
    const Eigen::MatrixXd& time_result) {
  optimized_trajectory_.clear();
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();
  CHECK_EQ(states_size, times_size);
  CHECK_EQ(states_size, controls_size);
  double relative_time = 0.0;
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_relative_time(relative_time);
    relative_time += time_result(0, i);
    point.set_v(state_result(3, i));
    // TODO(Jinyun) Evaluate how to set end states control input
    if (i == controls_size) {
      point.set_steer(0.0);
      point.set_a(0.0);
    } else {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }
    optimized_trajectory_.emplace_back(point);
  }
}

}  // namespace planning
}  // namespace apollo
