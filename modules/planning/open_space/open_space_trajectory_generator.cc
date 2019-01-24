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

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::VehicleState;

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
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    const VehicleState& vehicle_state, const std::vector<double>& XYbounds,
    const double& rotate_angle, const Vec2d& translate_origin,
    const std::vector<double>& end_pose,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  if (!vehicle_state.has_x() || XYbounds.size() == 0 || end_pose.size() == 0 ||
      obstacles_edges_num.cols() == 0 || obstacles_A.cols() == 0 ||
      obstacles_b.cols() == 0) {
    return Status(ErrorCode::PLANNING_ERROR, "Generator input data not ready");
  }

  // Generate Stop trajectory if init point close to destination
  if (IsInitPointNearDestination(stitching_trajectory.back(), end_pose,
                                 rotate_angle, translate_origin)) {
    AINFO << "Planning init point is close to destination, skip new "
             "trajectory generation.";

    return Status(ErrorCode::OK,
                  "Planning init point is close to destination, skip new "
                  "trajectory generation.");
  }

  // initial state
  stitching_trajectory_ = stitching_trajectory;
  planning_init_point_ = stitching_trajectory_.back();
  init_state_ = planning_init_point_.path_point();
  init_x_ = init_state_.x();
  init_y_ = init_state_.y();
  init_phi_ = init_state_.theta();
  init_v_ = planning_init_point_.v();
  // rotate and scale the state according to the origin point defined in
  // frame
  init_x_ -= translate_origin.x();
  init_y_ -= translate_origin.y();
  double tmp_x = init_x_;
  init_x_ =
      init_x_ * std::cos(-rotate_angle) - init_y_ * std::sin(-rotate_angle);
  init_y_ = tmp_x * std::sin(-rotate_angle) + init_y_ * std::cos(-rotate_angle);
  init_phi_ = common::math::NormalizeAngle(init_phi_ - rotate_angle);

  // initial control input
  init_steer_ = planning_init_point_.steer();
  init_a_ = planning_init_point_.a();

  Eigen::MatrixXd x0(4, 1);
  x0 << init_x_, init_y_, init_phi_, init_v_;

  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer_, init_a_;

  // final state
  Eigen::MatrixXd xF(4, 1);
  xF << end_pose[0], end_pose[1], end_pose[2], end_pose[3];

  // planning bound
  XYbounds_ = XYbounds;

  // obstacle num
  size_t obstacles_num = obstacles_vertices_vec.size();

  ADEBUG << "Start forming state warm start problem with configs setting : "
         << planner_open_space_config_.warm_start_config().ShortDebugString();

  // Warm Start (initial velocity is assumed to be 0 for now)
  HybridAStartResult result;

  if (warm_start_->Plan(x0(0, 0), x0(1, 0), x0(2, 0), xF(0, 0), xF(1, 0),
                        xF(2, 0), XYbounds_, obstacles_vertices_vec, &result)) {
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
    << planner_open_space_config_.\
      dual_variable_warm_start_config().ShortDebugString();

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
                    time_result_ds, XYbounds_, obstacles_vertices_vec);
  }
  // rescale the states to the world frame
  for (size_t i = 0; i < horizon_ + 1; ++i) {
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

  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  return Status::OK();
}

void OpenSpaceTrajectoryGenerator::UpdateTrajectory(
    apollo::common::Trajectory* trajectory_to_end) {
  trajectory_to_end->Clear();
  trajectory_to_end->mutable_trajectory_point()->CopyFrom(
      *(trajectory_to_end_.mutable_trajectory_point()));
}

void OpenSpaceTrajectoryGenerator::UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* open_space_debug) {
  open_space_debug->Clear();
  open_space_debug->CopyFrom(open_space_debug_);
}

void OpenSpaceTrajectoryGenerator::GetStitchingTrajectory(
    std::vector<common::TrajectoryPoint>* stitching_trajectory) {
  stitching_trajectory->clear();
  *stitching_trajectory = stitching_trajectory_;
}

void OpenSpaceTrajectoryGenerator::RecordDebugInfo(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    const Eigen::MatrixXd& dual_l_result_ds,
    const Eigen::MatrixXd& dual_n_result_ds,
    const Eigen::MatrixXd& state_result_ds,
    const Eigen::MatrixXd& control_result_ds,
    const Eigen::MatrixXd& time_result_ds, const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  // load warm start trajectory
  auto* warm_start_trajectory =
      open_space_debug_.mutable_warm_start_trajectory();
  for (size_t i = 0; i < horizon_; ++i) {
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
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < l_warm_up_cols; j++) {
      open_space_debug_.add_warm_start_dual_lambda(l_warm_up(j, i));
    }
  }
  size_t n_warm_up_cols = n_warm_up.rows();
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < n_warm_up_cols; j++) {
      open_space_debug_.add_warm_start_dual_miu(n_warm_up(j, i));
    }
  }

  // load optimized dual variables
  size_t dual_l_result_ds_cols = dual_l_result_ds.rows();
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < dual_l_result_ds_cols; j++) {
      open_space_debug_.add_optimized_dual_lambda(dual_l_result_ds(j, i));
    }
  }
  size_t dual_n_result_ds_cols = dual_n_result_ds.rows();
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < dual_n_result_ds_cols; j++) {
      open_space_debug_.add_optimized_dual_miu(dual_n_result_ds(j, i));
    }
  }

  double relative_time = 0;

  // load smoothed trajectory
  auto* smoothed_trajectory = open_space_debug_.mutable_smoothed_trajectory();
  for (size_t i = 0; i < horizon_; ++i) {
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
      state_result_ds(0, horizon_));
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_y(
      state_result_ds(1, horizon_));
  smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
      state_result_ds(2, horizon_));
  smoothed_point->mutable_trajectory_point()->set_v(
      state_result_ds(3, horizon_));
  relative_time += time_result_ds(0, horizon_);
  smoothed_point->mutable_trajectory_point()->set_relative_time(relative_time);
  // load xy boundary (xmin, xmax, ymin, ymax)
  open_space_debug_.add_xy_boundary(XYbounds[0]);
  open_space_debug_.add_xy_boundary(XYbounds[1]);
  open_space_debug_.add_xy_boundary(XYbounds[2]);
  open_space_debug_.add_xy_boundary(XYbounds[3]);

  // load obstacles
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    auto* obstacle_ptr = open_space_debug_.add_obstacles();
    size_t vertices_size = obstacle_vertices.size();
    for (size_t i = 0; i < vertices_size; ++i) {
      obstacle_ptr->add_vertices_x_coords(obstacle_vertices[i].x());
      obstacle_ptr->add_vertices_y_coords(obstacle_vertices[i].y());
    }
  }
}

void OpenSpaceTrajectoryGenerator::LoadTrajectory(
    const Eigen::MatrixXd& state_result_ds,
    const Eigen::MatrixXd& control_result_ds,
    const Eigen::MatrixXd& time_result_ds) {
  trajectory_to_end_.Clear();
  double relative_time = 0.0;
  for (size_t i = 0; i < horizon_ + 1; ++i) {
    auto* point = trajectory_to_end_.add_trajectory_point();
    point->mutable_path_point()->set_x(state_result_ds(0, i));
    point->mutable_path_point()->set_y(state_result_ds(1, i));
    point->mutable_path_point()->set_theta(state_result_ds(2, i));
    point->set_relative_time(relative_time);
    relative_time += time_result_ds(0, i);
    point->set_v(state_result_ds(3, i));
    point->set_steer(control_result_ds(0, i));
    point->set_a(control_result_ds(1, i));
  }
}

bool OpenSpaceTrajectoryGenerator::IsInitPointNearDestination(
    const common::TrajectoryPoint& planning_init_point,
    const std::vector<double>& end_pose, const double& rotate_angle,
    const Vec2d& translate_origin) {
  CHECK_EQ(end_pose.size(), 4);
  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  const apollo::common::PathPoint path_point = planning_init_point.path_point();
  double distance_to_init_point =
      (path_point.x() - end_pose_to_world_frame.x()) *
          (path_point.x() - end_pose_to_world_frame.x()) +
      (path_point.y() - end_pose_to_world_frame.y()) *
          (path_point.y() - end_pose_to_world_frame.y());

  if (distance_to_init_point <
      planner_open_space_config_.is_near_destination_threshold()) {
    AINFO << "init_point reach end_pose";
    return true;
  }

  return false;
}

}  // namespace planning
}  // namespace apollo
