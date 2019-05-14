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
  if (XYbounds.empty() || end_pose.empty() || obstacles_edges_num.cols() == 0 ||
      obstacles_A.cols() == 0 || obstacles_b.cols() == 0) {
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

  // Init trajectory point is the stitching point from last trajectory
  const common::TrajectoryPoint trajectory_stitching_point =
      stitching_trajectory.back();

  // init x, y, z would be rotated.
  double init_x = trajectory_stitching_point.path_point().x();
  double init_y = trajectory_stitching_point.path_point().y();
  double init_phi = trajectory_stitching_point.path_point().theta();

  // Rotate and scale the state
  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                       &init_phi);

  // Result container for warm start (initial velocity is assumed to be 0 for
  // now)
  HybridAStartResult result;

  if (warm_start_->Plan(init_x, init_y, init_phi, end_pose[0], end_pose[1],
                        end_pose[2], XYbounds, obstacles_vertices_vec,
                        &result)) {
    ADEBUG << "State warm start problem solved successfully!";
  } else {
    ADEBUG << "State warm start problem failed to solve";
    return Status(ErrorCode::PLANNING_ERROR,
                  "State warm start problem failed to solve");
  }

  // Result Containers for distance approach trajectory smoothing problem
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;
  Eigen::MatrixXd dual_l_result_ds;
  Eigen::MatrixXd dual_n_result_ds;

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

  const double init_steer = trajectory_stitching_point.steer();
  const double init_a = trajectory_stitching_point.a();
  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer, init_a;

  const double init_v = trajectory_stitching_point.v();

  if (!GenerateDistanceApproachTraj(
          xWS, uWS, XYbounds, obstacles_edges_num, obstacles_A, obstacles_b,
          obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
          &control_result_ds, &time_result_ds, &l_warm_up, &n_warm_up,
          &dual_l_result_ds, &dual_n_result_ds)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "distance approach smoothing problem failed to solve");
  }

  // record debug info
  if (FLAGS_enable_record_debug) {
    open_space_debug_.Clear();
    RecordDebugInfo(trajectory_stitching_point, translate_origin, rotate_angle,
                    end_pose, xWS, uWS, l_warm_up, n_warm_up, dual_l_result_ds,
                    dual_n_result_ds, state_result_ds, control_result_ds,
                    time_result_ds, XYbounds, obstacles_vertices_vec);
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

void OpenSpaceTrajectoryOptimizer::RecordDebugInfo(
    const common::TrajectoryPoint& trajectory_stitching_point,
    const Vec2d& translate_origin, const double rotate_angle,
    const std::vector<double>& end_pose, const Eigen::MatrixXd& xWS,
    const Eigen::MatrixXd& uWS, const Eigen::MatrixXd& l_warm_up,
    const Eigen::MatrixXd& n_warm_up, const Eigen::MatrixXd& dual_l_result_ds,
    const Eigen::MatrixXd& dual_n_result_ds,
    const Eigen::MatrixXd& state_result_ds,
    const Eigen::MatrixXd& control_result_ds,
    const Eigen::MatrixXd& time_result_ds, const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  // load information about trajectory stitching point

  open_space_debug_.mutable_trajectory_stitching_point()->CopyFrom(
      trajectory_stitching_point);
  // load translation origin and heading angle
  auto* roi_shift_point = open_space_debug_.mutable_roi_shift_point();
  // pathpoint
  roi_shift_point->mutable_path_point()->set_x(translate_origin.x());
  roi_shift_point->mutable_path_point()->set_y(translate_origin.y());
  roi_shift_point->mutable_path_point()->set_theta(rotate_angle);

  // load end_pose into debug
  auto* end_point = open_space_debug_.mutable_end_point();
  end_point->mutable_path_point()->set_x(end_pose[0]);
  end_point->mutable_path_point()->set_y(end_pose[1]);
  end_point->mutable_path_point()->set_theta(end_pose[2]);
  end_point->set_v(end_pose[3]);

  // load warm start trajectory
  size_t horizon = uWS.cols();
  auto* warm_start_trajectory =
      open_space_debug_.mutable_warm_start_trajectory();
  for (size_t i = 0; i < horizon; ++i) {
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
      xWS(0, horizon));
  warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_y(
      xWS(1, horizon));
  warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
      xWS(2, horizon));
  warm_start_point->mutable_trajectory_point()->set_v(xWS(3, horizon));

  // load warm start dual variables
  size_t l_warm_up_cols = l_warm_up.rows();
  for (size_t i = 0; i < horizon; ++i) {
    for (size_t j = 0; j < l_warm_up_cols; j++) {
      open_space_debug_.add_warm_start_dual_lambda(l_warm_up(j, i));
    }
  }
  size_t n_warm_up_cols = n_warm_up.rows();
  for (size_t i = 0; i < horizon; ++i) {
    for (size_t j = 0; j < n_warm_up_cols; j++) {
      open_space_debug_.add_warm_start_dual_miu(n_warm_up(j, i));
    }
  }

  // load optimized dual variables
  size_t dual_l_result_ds_cols = dual_l_result_ds.rows();
  for (size_t i = 0; i < horizon; ++i) {
    for (size_t j = 0; j < dual_l_result_ds_cols; j++) {
      open_space_debug_.add_optimized_dual_lambda(dual_l_result_ds(j, i));
    }
  }
  size_t dual_n_result_ds_cols = dual_n_result_ds.rows();
  for (size_t i = 0; i < horizon; ++i) {
    for (size_t j = 0; j < dual_n_result_ds_cols; j++) {
      open_space_debug_.add_optimized_dual_miu(dual_n_result_ds(j, i));
    }
  }

  double relative_time = 0;

  // load smoothed trajectory
  auto* smoothed_trajectory = open_space_debug_.mutable_smoothed_trajectory();
  for (size_t i = 0; i < horizon; ++i) {
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
    relative_time += time_result_ds(0, i);
    smoothed_point->mutable_trajectory_point()->set_relative_time(
        relative_time);
  }
  auto* smoothed_point = smoothed_trajectory->add_vehicle_motion_point();
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_x(
      state_result_ds(0, horizon));
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_y(
      state_result_ds(1, horizon));
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
      state_result_ds(2, horizon));
  smoothed_point->mutable_trajectory_point()->set_v(
      state_result_ds(3, horizon));
  relative_time += time_result_ds(0, horizon);
  smoothed_point->mutable_trajectory_point()->set_relative_time(relative_time);

  // load xy boundary (xmin, xmax, ymin, ymax)
  open_space_debug_.add_xy_boundary(XYbounds[0]);
  open_space_debug_.add_xy_boundary(XYbounds[1]);
  open_space_debug_.add_xy_boundary(XYbounds[2]);
  open_space_debug_.add_xy_boundary(XYbounds[3]);

  // load obstacles
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    auto* obstacle_ptr = open_space_debug_.add_obstacles();
    for (const auto& vertex : obstacle_vertices) {
      obstacle_ptr->add_vertices_x_coords(vertex.x());
      obstacle_ptr->add_vertices_y_coords(vertex.y());
    }
  }
}

void OpenSpaceTrajectoryOptimizer::UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* open_space_debug) {
  open_space_debug->MergeFrom(open_space_debug_);
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
  double relative_s = 0.0;
  common::math::Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_relative_time(relative_time);
    point.set_v(state_result(3, i));
    relative_time += time_result(0, i);
    common::math::Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
    // TODO(Jinyun) Evaluate how to set end states control input
    if (i == controls_size) {
      point.set_steer(0.0);
      point.set_a(0.0);
    } else {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }
    optimized_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}

void OpenSpaceTrajectoryOptimizer::UseWarmStartAsResult(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds) {
  AERROR << "Use warm start as trajectory output";

  *state_result_ds = xWS;
  *control_result_ds = uWS;
  *dual_l_result_ds = l_warm_up;
  *dual_n_result_ds = n_warm_up;

  control_result_ds->conservativeResize(control_result_ds->rows(),
                                        control_result_ds->cols() + 1);
  control_result_ds->col(control_result_ds->cols() - 1) << 0.0, 0.0;

  size_t time_result_horizon = xWS.cols();
  *time_result_ds = Eigen::MatrixXd::Constant(
      1, time_result_horizon, config_.planner_open_space_config().delta_t());
}

bool OpenSpaceTrajectoryOptimizer::GenerateDistanceApproachTraj(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const std::vector<double>& XYbounds,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    const Eigen::MatrixXd& last_time_u, const double init_v,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds) {
  size_t horizon = xWS.cols() - 1;
  Eigen::MatrixXd x0(4, 1);
  x0 << xWS(0, 0), xWS(1, 0), xWS(2, 0), init_v;

  Eigen::MatrixXd xF(4, 1);
  xF << xWS(0, horizon), xWS(1, horizon), xWS(2, horizon), xWS(3, horizon);

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

  // Dual variable warm start for distance approach problem
  if (FLAGS_use_dual_variable_warm_start) {
    if (dual_variable_warm_start_->Solve(
            horizon, ts, ego, obstacles_num, obstacles_edges_num, obstacles_A,
            obstacles_b, xWS, l_warm_up, n_warm_up)) {
      ADEBUG << "Dual variable problem solved successfully!";
    } else {
      ADEBUG << "Dual variable problem failed to solve";
      return false;
    }
  } else {
    *l_warm_up =
        0.5 * Eigen::MatrixXd::Ones(obstacles_edges_num.sum(), horizon + 1);
    *n_warm_up = 0.5 * Eigen::MatrixXd::Ones(4 * obstacles_num, horizon + 1);
  }

  // Distance approach trajectory smoothing
  if (distance_approach_->Solve(
          x0, xF, last_time_u, horizon, ts, ego, xWS, uWS, *l_warm_up,
          *n_warm_up, XYbounds, obstacles_num, obstacles_edges_num, obstacles_A,
          obstacles_b, state_result_ds, control_result_ds, time_result_ds,
          dual_l_result_ds, dual_n_result_ds)) {
    ADEBUG << "Distance approach problem solved successfully!";
  } else {
    ADEBUG << "Distance approach problem failed to solve";
    if (FLAGS_enable_smoother_failsafe) {
      UseWarmStartAsResult(xWS, uWS, *l_warm_up, *n_warm_up, state_result_ds,
                           control_result_ds, time_result_ds, dual_l_result_ds,
                           dual_n_result_ds);
    } else {
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
